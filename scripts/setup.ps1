param(
    [switch]$Update,
    [switch]$SkipValidation
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $ProjectRoot

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "Git is required. Install Git, reopen PowerShell, and rerun this script."
}
if (-not (Get-Command conda -ErrorAction SilentlyContinue)) {
    throw "Conda is required. Install Miniforge, open a Miniforge/Conda PowerShell, and rerun this script."
}

$SubmodulePath = Join-Path $ProjectRoot "third_party\RFT-SiM"
$ExpectedSubmoduleCommit = "303283fae075cae4101ee3af102a36a4a5775998"
$SubmoduleReady = $false
if (Test-Path -LiteralPath (Join-Path $SubmodulePath ".git")) {
    $SubmoduleCommit = git -C $SubmodulePath rev-parse HEAD
    $SubmoduleReady = (
        $LASTEXITCODE -eq 0 -and
        $SubmoduleCommit.Trim() -eq $ExpectedSubmoduleCommit
    )
}
if ($SubmoduleReady) {
    Write-Host "[1/4] Pinned RFT-SiM submodule is already initialized."
} else {
    Write-Host "[1/4] Initializing the pinned RFT-SiM submodule..."
    git submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) {
        throw "Git submodule initialization failed. See GUIDANCE.md troubleshooting."
    }
}

$EnvironmentName = "lizard_rft"
$EnvironmentPaths = (conda env list --json | ConvertFrom-Json).envs
$EnvironmentExists = $false
foreach ($EnvironmentPath in $EnvironmentPaths) {
    if ((Split-Path -Leaf $EnvironmentPath) -eq $EnvironmentName) {
        $EnvironmentExists = $true
        break
    }
}

if (-not $EnvironmentExists) {
    Write-Host "[2/4] Creating Conda environment '$EnvironmentName'..."
    conda env create --file environment.yml
} elseif ($Update) {
    Write-Host "[2/4] Updating Conda environment '$EnvironmentName'..."
    conda env update --name $EnvironmentName --file environment.yml --prune
} else {
    Write-Host "[2/4] Reusing existing Conda environment '$EnvironmentName'."
    Write-Host "      Pass -Update to synchronize it with environment.yml."
}
if ($LASTEXITCODE -ne 0) {
    throw "Conda environment creation/update failed."
}

Write-Host "[3/4] Checking pinned runtime imports..."
conda run --no-capture-output --name $EnvironmentName python -c "import mujoco, numpy, open3d, pymeshlab, cv2, imageio; print('MuJoCo', mujoco.__version__); print('NumPy', numpy.__version__); print('Open3D', open3d.__version__)"
if ($LASTEXITCODE -ne 0) {
    throw "Runtime import check failed."
}

if (-not $SkipValidation) {
    Write-Host "[4/4] Running project validation..."
    conda run --no-capture-output --name $EnvironmentName python scripts/validate_project.py
    if ($LASTEXITCODE -ne 0) {
        throw "Project validation failed."
    }
} else {
    Write-Host "[4/4] Validation skipped by request."
}

Write-Host ""
Write-Host "Deployment complete."
Write-Host "Activate with: conda activate $EnvironmentName"
Write-Host "Read GUIDANCE.md, or open the checked-in videos under docs/media/."
