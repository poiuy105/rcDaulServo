# GitHub CLI 快捷脚本
# 用于查看GitHub Actions编译状态

$GH_PATH = "C:\Program Files\GitHub CLI\gh.exe"

function Show-BuildStatus {
    param(
        [int]$Limit = 5
    )
    & $GH_PATH run list --limit $Limit
}

function Watch-Build {
    param(
        [string]$RunId
    )
    & $GH_PATH run watch $RunId
}

function View-RunDetails {
    param(
        [string]$RunId
    )
    & $GH_PATH run view $RunId
}

# 主执行
Write-Host "GitHub Actions 编译状态：`n" -ForegroundColor Green
Show-BuildStatus -Limit 5
