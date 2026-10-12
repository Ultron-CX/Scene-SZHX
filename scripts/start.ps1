[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$sceneSemRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$sceneSemPython = Join-Path $sceneSemRoot '.venv/Scripts/python.exe'
if (!(Test-Path -LiteralPath $sceneSemPython)) { throw 'SceneSem .venv is missing; see README.md for environment preparation.' }
# No automatic port fallback, process killing, or browser refresh.
Push-Location $sceneSemRoot
try {
    & $sceneSemPython -X utf8 (Join-Path $PSScriptRoot 'serve_annotation_tool.py')
    if ($LASTEXITCODE -ne 0) { throw "SceneSem exited ($LASTEXITCODE). If 18876 is occupied, verify its owner before stopping it." }
} finally { Pop-Location }
