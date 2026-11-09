@echo off
chcp 65001 >nul
title ISILAB UAV - TEST 2 GIA THIET DIEU KHIEN
cd /d "%~dp0"
python test_dual_hypotheses.py
pause

