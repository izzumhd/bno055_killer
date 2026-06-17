@echo off
title Git Auto Push

echo =========================
echo      GIT AUTO PUSH
echo =========================
echo.

:: masuk folder tempat .bat berada
cd /d "%~dp0"

echo Pull update terbaru...
git pull

echo.
echo Add semua file...
git add .

echo.
set /p msg=Commit message: 

echo.
echo Commit perubahan...
git commit -m "%msg%"

echo.
echo Push ke GitHub...
git push

echo.
echo Selesai!
pause
