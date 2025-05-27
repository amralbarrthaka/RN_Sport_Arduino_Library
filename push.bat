@echo off
REM Ask for commit message
set /p comment=Enter commit message: 

REM Run git commands
git add .
git commit -m "%comment%"
git push

REM Keep terminal open
pause
