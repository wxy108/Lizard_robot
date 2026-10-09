#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

UPDATE=0
SKIP_VALIDATION=0
for argument in "$@"; do
    case "${argument}" in
        --update) UPDATE=1 ;;
        --skip-validation) SKIP_VALIDATION=1 ;;
        *)
            echo "Unknown argument: ${argument}" >&2
            echo "Usage: bash scripts/setup.sh [--update] [--skip-validation]" >&2
            exit 2
            ;;
    esac
done

command -v conda >/dev/null 2>&1 || {
    echo "Conda is required. Install Miniforge, initialize your shell, and rerun." >&2
    exit 1
}

if [[ ! -f third_party/RFT-SiM.snapshot.json || ! -f third_party/RFT-SiM/README.md ]]; then
    echo "Bundled RFT-SiM snapshot is missing. Download a fresh clone or ZIP of Lizard_robot." >&2
    exit 1
fi
echo "[1/4] Using the bundled RFT-SiM snapshot (no upstream download)."

ENVIRONMENT_NAME="lizard_rft"
if conda env list | awk '{print $1}' | grep -Fxq "${ENVIRONMENT_NAME}"; then
    if [[ "${UPDATE}" -eq 1 ]]; then
        echo "[2/4] Updating Conda environment '${ENVIRONMENT_NAME}'..."
        conda env update --name "${ENVIRONMENT_NAME}" --file environment.yml --prune
    else
        echo "[2/4] Reusing existing Conda environment '${ENVIRONMENT_NAME}'."
        echo "      Pass --update to synchronize it with environment.yml."
    fi
else
    echo "[2/4] Creating Conda environment '${ENVIRONMENT_NAME}'..."
    conda env create --file environment.yml
fi

echo "[3/4] Checking pinned runtime imports..."
conda run --no-capture-output --name "${ENVIRONMENT_NAME}" \
    python -c "import mujoco, numpy, open3d, pymeshlab, cv2, imageio; print('MuJoCo', mujoco.__version__); print('NumPy', numpy.__version__); print('Open3D', open3d.__version__)"
conda run --no-capture-output --name "${ENVIRONMENT_NAME}" python scripts/verify_rft_snapshot.py

if [[ "$(uname -s)" == "Linux" && -z "${DISPLAY:-}" ]]; then
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
    echo "      Headless Linux detected; using MUJOCO_GL=${MUJOCO_GL}."
fi

if [[ "${SKIP_VALIDATION}" -eq 0 ]]; then
    echo "[4/4] Running project validation..."
    conda run --no-capture-output --name "${ENVIRONMENT_NAME}" \
        python scripts/validate_project.py
else
    echo "[4/4] Validation skipped by request."
fi

echo
echo "Deployment complete."
echo "Activate with: conda activate ${ENVIRONMENT_NAME}"
echo "Read GUIDANCE.md, or open the checked-in videos under docs/media/."
