@echo off
chcp 65001 >nul
title ISILAB UAV - PIPELINE DIAGNOSTIC TOOL
cd /d "%~dp0"
python diagnose_pipeline.py
pause

