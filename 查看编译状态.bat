@echo off
chcp 65001 >nul
echo GitHub Actions 编译状态：
echo =====================================
"C:\Program Files\GitHub CLI\gh.exe" run list --limit 5
echo.
echo 按任意键退出...
pause >nul
