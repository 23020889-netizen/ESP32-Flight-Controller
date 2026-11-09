#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UNIT TEST SUITE FOR PROTOCOL V2 BOUNDED RC PARSER
Validates Section 3.5 & Section 8.1 of ANALOG_RC_PIPELINE_REVIEW_V2_6.md
"""

import math
import sys

class RcPacket:
    def __init__(self, session_id=0, seq=0, throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.session_id = session_id
        self.seq = seq
        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class BoundedRcParserV2:
    def __init__(self):
        self.active_session_id = 0
        self.last_valid_seq = 0
        self.accepted_count = 0
        self.stale_count = 0
        self.rejected_count = 0
        self.current_snapshot = None

    def is_seq_newer(self, candidate, previous):
        # RFC 1982 signed 32-bit window
        diff = (candidate - previous) & 0xFFFFFFFF
        if diff & 0x80000000:
            return False  # Candidate is older or equal
        return diff > 0

    def parse_payload(self, raw_bytes: bytes, source: str = "MQTT") -> bool:
        MAX_LEN = 128
        if not raw_bytes or len(raw_bytes) >= MAX_LEN:
            self.rejected_count += 1
            return False

        # Reject embedded NUL
        if b'\x00' in raw_bytes:
            self.rejected_count += 1
            return False

        try:
            text = raw_bytes.decode('ascii')
        except UnicodeDecodeError:
            self.rejected_count += 1
            return False

        text = text.rstrip('\r\n \t')
        if not text:
            self.rejected_count += 1
            return False

        if not text.startswith("RC,") and not text.startswith("rc,"):
            self.rejected_count += 1
            return False

        tokens = text[3:].split(',')
        # Protocol V2 requires exactly 7 fields: 2, session_id, seq, thr, roll, pitch, yaw
        if len(tokens) != 7:
            self.rejected_count += 1
            return False

        try:
            proto_ver = int(tokens[0].strip())
            if proto_ver != 2:
                self.rejected_count += 1
                return False

            session_id = int(tokens[1].strip())
            seq = int(tokens[2].strip())
            thr = float(tokens[3].strip())
            roll = float(tokens[4].strip())
            pitch = float(tokens[5].strip())
            yaw = float(tokens[6].strip())
        except ValueError:
            self.rejected_count += 1
            return False

        # Check finiteness (reject NaN, Inf)
        for v in (thr, roll, pitch, yaw):
            if not math.isfinite(v):
                self.rejected_count += 1
                return False

        # Range validation
        if not (0.0 <= thr <= 100.0 and -100.0 <= roll <= 100.0 and -100.0 <= pitch <= 100.0 and -100.0 <= yaw <= 100.0):
            self.rejected_count += 1
            return False

        # Session and sequence tracking
        if session_id != self.active_session_id:
            self.active_session_id = session_id
            self.last_valid_seq = seq
        else:
            if not self.is_seq_newer(seq, self.last_valid_seq):
                self.stale_count += 1
                return False
            self.last_valid_seq = seq

        self.current_snapshot = RcPacket(session_id, seq, thr, roll, pitch, yaw)
        self.accepted_count += 1
        return True

def run_all_tests():
    parser = BoundedRcParserV2()
    test_cases = [
        # (name, payload, expected_accept, expected_reason)
        ("T1: Normal packet V2", b"RC,2,1001,1,50.0,0.0,0.0,0.0\n", True, "Accept"),
        ("T2: Valid stick range", b"RC,2,1001,2,98.0,-10.0,5.0,22.0\r\n", True, "Accept"),
        ("T3: Reject NaN throttle", b"RC,2,1001,3,nan,0.0,0.0,0.0\n", False, "NaN rejection"),
        ("T4: Reject Inf throttle", b"RC,2,1001,4,inf,0.0,0.0,0.0\n", False, "Inf rejection"),
        ("T5: Reject Out of Range throttle 101%", b"RC,2,1001,5,101.0,0.0,0.0,0.0\n", False, "Range rejection"),
        ("T6: Reject Malformed token '50abc'", b"RC,2,1001,6,50abc,0.0,0.0,0.0\n", False, "Syntax rejection"),
        ("T7: Reject Missing field (4 fields only)", b"RC,2,1001,7,50.0,0.0,0.0\n", False, "Field count rejection"),
        ("T8: Reject Extra field", b"RC,2,1001,8,50.0,0.0,0.0,0.0,extra\n", False, "Field count rejection"),
        ("T9: Reject Payload too long", b"RC,2,1001,9," + b"1" * 150 + b"\n", False, "Buffer boundary rejection"),
        ("T10: Reject Embedded NUL", b"RC,2,1001,10,50.0,\x00.0,0.0,0.0\n", False, "Embedded NUL rejection"),
        ("T11: Reject Stale / Duplicate sequence (seq 2 again in session 1001)", b"RC,2,1001,2,50.0,0.0,0.0,0.0\n", False, "Stale sequence rejection"),
        ("T12: Accept New Session after browser reload (Session 2002, seq 1)", b"RC,2,2002,1,50.0,0.0,0.0,0.0\n", True, "Session reset acceptance"),
        ("T13: Accept Sequential packet in Session 2002 (seq 2)", b"RC,2,2002,2,75.0,10.0,-5.0,0.0\n", True, "Sequential acceptance"),
    ]

    print("=" * 70)
    print("RUNNING PROTOCOL V2 BOUNDED RC PARSER UNIT TESTS (TABLE 3.5 & 8.1)")
    print("=" * 70)

    passed = 0
    for name, payload, expected, reason in test_cases:
        result = parser.parse_payload(payload)
        status = "PASS" if result == expected else "FAIL"
        if result == expected:
            passed += 1
        print(f"[{status}] {name} => Result: {result} (Expected: {expected}) - {reason}")

    print("-" * 70)
    print(f"Summary: {passed}/{len(test_cases)} tests passed.")
    print(f"Accepted: {parser.accepted_count}, Stale Dropped: {parser.stale_count}, Rejected Format: {parser.rejected_count}")
    print("=" * 70)

    if passed == len(test_cases):
        print("ALL PROTOCOL V2 UNIT TESTS PASSED SUCCESSFULLY!")
        return 0
    else:
        print("SOME TESTS FAILED!")
        return 1

if __name__ == "__main__":
    sys.exit(run_all_tests())

