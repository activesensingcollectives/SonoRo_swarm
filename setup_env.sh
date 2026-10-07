#!/usr/bin/env bash
# ==============================================================================
# SonoRo Swarm - Automated Environment & Dependencies Setup Script
# Based on docs/source/installation.rst
# ==============================================================================

set -eo pipefail

# Text formatting
BOLD="\033[1m"
GREEN="\033[0;32m"
BLUE="\033[0;34m"
YELLOW="\033[0;33m"
RED="\033[0;31m"
NC="\033[0m" # No Color

# Default parameters (aligned with installation.rst & fleet_manager.py)
DEFAULT_ENV_NAME="sonoro"
DEFAULT_PYTHON_VERSION="3.11.2"
DEFAULT_REQUIREMENTS="requirements.txt"

ENV_NAME="${DEFAULT_ENV_NAME}"
PYTHON_VERSION="${DEFAULT_PYTHON_VERSION}"
REQUIREMENTS_FILE="${DEFAULT_REQUIREMENTS}"
SKIP_APT=false
SKIP_MINIFORGE=false
SKIP_UDEV=false
NON_INTERACTIVE=false

# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------
log_info() {
    echo -e "${BLUE}${BOLD}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}${BOLD}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}${BOLD}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}${BOLD}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
Usage: bash setup_env.sh [OPTIONS]

Automated setup for SonoRo Swarm Conda environment and dependencies (from docs/source/installation.rst).

Options:
  -n, --env-name <NAME>      Conda environment name (default: ${DEFAULT_ENV_NAME})
  -p, --python <VERSION>     Python version for Conda env (default: ${DEFAULT_PYTHON_VERSION})
  -r, --requirements <FILE>  Path to requirements.txt (default: ${DEFAULT_REQUIREMENTS})
  -y, --yes                  Run non-interactively (accept all prompts)
      --skip-apt             Skip installing system apt-get packages
      --skip-miniforge       Skip Miniforge/Conda installation check
      --skip-udev            Skip creating udev rules & user group permissions
  -h, --help                 Show this help message and exit

Examples:
  bash setup_env.sh
  bash setup_env.sh --env-name robat --yes
  bash setup_env.sh --env-name my_env --python 3.11.2
EOF
}

# ------------------------------------------------------------------------------
# Parse Command Line Arguments
# ------------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--name|--env-name)
            ENV_NAME="$2"
            shift 2
            ;;
        -p|--python)
            PYTHON_VERSION="$2"
            shift 2
            ;;
        -r|--requirements)
            REQUIREMENTS_FILE="$2"
            shift 2
            ;;
        -y|--yes)
            NON_INTERACTIVE=true
            shift
            ;;
        --skip-apt)
            SKIP_APT=true
            shift
            ;;
        --skip-miniforge)
            SKIP_MINIFORGE=true
            shift
            ;;
        --skip-udev)
            SKIP_UDEV=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown argument: $1"
            show_help
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if [[ ! -f "${REQUIREMENTS_FILE}" && -f "${SCRIPT_DIR}/${REQUIREMENTS_FILE}" ]]; then
    REQUIREMENTS_FILE="${SCRIPT_DIR}/${REQUIREMENTS_FILE}"
fi

echo -e "${BOLD}================================================================${NC}"
echo -e "${BOLD}     SonoRo Swarm - Environment & Dependencies Installer       ${NC}"
echo -e "${BOLD}================================================================${NC}"
log_info "Target Conda Environment : ${BOLD}${ENV_NAME}${NC}"
log_info "Python Version           : ${BOLD}${PYTHON_VERSION}${NC}"
log_info "Requirements File        : ${BOLD}${REQUIREMENTS_FILE}${NC}"
log_info "Architecture             : ${BOLD}$(uname -m)${NC}"
echo ""

# ------------------------------------------------------------------------------
# 1. System APT Dependencies (Step 4 in installation.rst)
# ------------------------------------------------------------------------------
if [ "${SKIP_APT}" = false ]; then
    log_info "Checking system (apt) dependencies..."
    if command -v apt-get >/dev/null 2>&1; then
        APT_PACKAGES=(
            libasound-dev
            libportaudio2
            portaudio19-dev
            git
            gcc
            g++
            make
            pkg-config
            libusb-1.0-0
            libusb-1.0-0-dev
            python3-dev
            psmisc
        )
        if command -v sudo >/dev/null 2>&1; then
            log_info "Installing system packages via apt-get: ${APT_PACKAGES[*]}"
            sudo apt-get update -y
            sudo apt-get install -y "${APT_PACKAGES[@]}"
            log_success "System packages installed successfully."
        else
            log_warn "'sudo' not found. If running as non-root, ensure PortAudio and ALSA libraries are installed."
        fi
    else
        log_warn "apt-get not found on this system. Skipping system package installation."
    fi
else
    log_info "Skipping APT system dependencies (--skip-apt specified)."
fi
echo ""

# ------------------------------------------------------------------------------
# 2. Conda / Miniforge Detection and Installation (Step 2 in installation.rst)
# ------------------------------------------------------------------------------
find_conda_sh() {
    for c_path in \
        "${CONDA_EXE%/bin/conda}" \
        "${HOME}/miniforge3" \
        "${HOME}/miniforge-pypy3" \
        "${HOME}/miniconda3" \
        "${HOME}/miniforge" \
        "${HOME}/mambaforge" \
        "${HOME}/anaconda3" \
        "/opt/conda" \
        "${HOME}/.conda"; do
        if [ -n "${c_path}" ] && [ -f "${c_path}/etc/profile.d/conda.sh" ]; then
            echo "${c_path}/etc/profile.d/conda.sh"
            return 0
        fi
    done
    return 1
}

CONDA_SH_PATH=""
if command -v conda >/dev/null 2>&1; then
    log_info "Found conda in PATH: $(which conda)"
    CONDA_SH_PATH=$(find_conda_sh || true)
fi

if [ -z "${CONDA_SH_PATH}" ]; then
    CONDA_SH_PATH=$(find_conda_sh || true)
fi

if [ -z "${CONDA_SH_PATH}" ] && [ "${SKIP_MINIFORGE}" = false ]; then
    log_warn "Conda not found. Installing Miniforge as per installation.rst..."
    ARCH="$(uname -m)"
    case "${ARCH}" in
        aarch64|arm64)
            MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
            ;;
        x86_64)
            MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh"
            ;;
        armv7l)
            MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-armv7l.sh"
            ;;
        *)
            MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${ARCH}.sh"
            ;;
    esac

    INSTALLER="/tmp/miniforge_installer.sh"
    log_info "Downloading Miniforge installer from: ${MINIFORGE_URL}"
    if command -v wget >/dev/null 2>&1; then
        wget -O "${INSTALLER}" "${MINIFORGE_URL}"
    elif command -v curl >/dev/null 2>&1; then
        curl -fsSL -o "${INSTALLER}" "${MINIFORGE_URL}"
    else
        log_error "Neither wget nor curl is installed. Cannot download Miniforge."
        exit 1
    fi

    INSTALL_PREFIX="${HOME}/miniforge3"
    log_info "Installing Miniforge to ${INSTALL_PREFIX}..."
    bash "${INSTALLER}" -b -p "${INSTALL_PREFIX}"
    rm -f "${INSTALLER}"

    CONDA_SH_PATH="${INSTALL_PREFIX}/etc/profile.d/conda.sh"
    "${INSTALL_PREFIX}/bin/conda" init bash || true
    log_success "Miniforge installed successfully."
fi

if [ -n "${CONDA_SH_PATH}" ] && [ -f "${CONDA_SH_PATH}" ]; then
    log_info "Sourcing Conda from ${CONDA_SH_PATH}"
    # shellcheck source=/dev/null
    source "${CONDA_SH_PATH}"
else
    log_error "Could not source conda.sh. Please install Conda / Miniforge manually or re-run without --skip-miniforge."
    exit 1
fi
echo ""

# ------------------------------------------------------------------------------
# 3. Create / Verify Conda Environment (Step 2 in installation.rst)
# ------------------------------------------------------------------------------
log_info "Configuring Conda environment: '${ENV_NAME}' (Python ${PYTHON_VERSION})..."

if conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
    log_info "Conda environment '${ENV_NAME}' already exists."
else
    log_info "Creating conda environment '${ENV_NAME}' with python=${PYTHON_VERSION}..."
    conda create -y -n "${ENV_NAME}" python="${PYTHON_VERSION}"
    log_success "Conda environment '${ENV_NAME}' created."
fi

# Activate the target environment
conda activate "${ENV_NAME}"
log_info "Active Python: $(which python3) ($(python3 --version))"
echo ""

# ------------------------------------------------------------------------------
# 4. Install Python Requirements (Step 3 & 4 in installation.rst)
# ------------------------------------------------------------------------------
log_info "Upgrading pip, setuptools, and wheel..."
pip install --upgrade pip setuptools wheel

if [ -f "${REQUIREMENTS_FILE}" ]; then
    log_info "Installing dependencies from ${REQUIREMENTS_FILE}..."
    pip install -r "${REQUIREMENTS_FILE}"
    log_success "Requirements installed from ${REQUIREMENTS_FILE}."
else
    log_warn "Requirements file '${REQUIREMENTS_FILE}' not found! Installing core requirements directly..."
    pip install "numpy<2" scipy pandas sounddevice soundfile pyroomacoustics librosa psutil pyserial pyusb hydra-core
fi

# Step 4: Install Thymiodirect from EPFL Mobots GitHub repository
log_info "Installing / updating thymiodirect from GitHub (installation.rst step 4)..."
pip install "thymiodirect@git+https://github.com/epfl-mobots/thymio-python"
log_success "thymiodirect installed successfully."
echo ""

# ------------------------------------------------------------------------------
# 5. Hardware Permissions & Udev Rules (ReSpeaker & Thymio)
# ------------------------------------------------------------------------------
if [ "${SKIP_UDEV}" = false ]; then
    log_info "Configuring hardware permissions and udev rules..."
    if command -v sudo >/dev/null 2>&1; then
        # 1. Add current user to dialout, audio, and video groups
        CURRENT_USER="$(id -un)"
        log_info "Adding user '${CURRENT_USER}' to groups 'dialout', 'audio'..."
        sudo usermod -aG dialout,audio "${CURRENT_USER}" 2>/dev/null || true

        # 2. ReSpeaker USB udev rule (Vendor ID: 2886, Product ID: 0018)
        RESPEAKER_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666", GROUP="audio"'
        sudo mkdir -p /etc/udev/rules.d
        echo "${RESPEAKER_RULE}" | sudo tee /etc/udev/rules.d/99-respeaker.rules >/dev/null
        log_info "Created /etc/udev/rules.d/99-respeaker.rules"

        # 3. Thymio serial/USB udev rule (Vendor ID: 0617, Product ID: 000a)
        THYMIO_RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="0617", ATTRS{idProduct}=="000a", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0617", ATTRS{idProduct}=="000a", MODE="0666", GROUP="dialout"'
        echo "${THYMIO_RULE}" | sudo tee /etc/udev/rules.d/99-thymio.rules >/dev/null
        log_info "Created /etc/udev/rules.d/99-thymio.rules"

        # 4. Reload udev rules
        sudo udevadm control --reload-rules 2>/dev/null || true
        sudo udevadm trigger 2>/dev/null || true
        log_success "Udev rules reloaded."
    else
        log_warn "'sudo' not available. Skipping udev rule creation."
    fi
else
    log_info "Skipping udev rules (--skip-udev specified)."
fi
echo ""

# ------------------------------------------------------------------------------
# 6. Verification / Smoke Test
# ------------------------------------------------------------------------------
log_info "Verifying installed packages..."

VERIFY_SCRIPT="
import sys

modules = [
    ('numpy', 'numpy'),
    ('scipy', 'scipy'),
    ('pandas', 'pandas'),
    ('sounddevice', 'sounddevice'),
    ('soundfile', 'soundfile'),
    ('thymiodirect', 'thymiodirect'),
    ('pyusb (usb.core)', 'usb.core'),
    ('hydra', 'hydra'),
    ('omegaconf', 'omegaconf'),
    ('netifaces', 'netifaces'),
    ('psutil', 'psutil'),
    ('matplotlib', 'matplotlib'),
]

all_ok = True
for name, mod in modules:
    try:
        __import__(mod)
        print(f'  [OK] {name}')
    except Exception as e:
        print(f'  [FAIL] {name}: {e}')
        all_ok = False

if not all_ok:
    sys.exit(1)
"

if python3 -c "${VERIFY_SCRIPT}"; then
    log_success "All core Python packages imported successfully!"
else
    log_error "Some packages failed to import. Review errors above."
    exit 1
fi
echo ""

# ------------------------------------------------------------------------------
# Completion Banner
# ------------------------------------------------------------------------------
echo -e "${BOLD}================================================================${NC}"
echo -e "${GREEN}${BOLD}      SonoRo Swarm setup completed successfully!                ${NC}"
echo -e "${BOLD}================================================================${NC}"
echo -e "To activate your environment in any shell, run:"
echo -e "  ${BOLD}${BLUE}conda activate ${ENV_NAME}${NC}"
echo ""
echo -e "To run a swarm experiment:"
echo -e "  ${BOLD}python fleet_manager.py --ips-file robots.txt --sync --time-sync --conda-env ${ENV_NAME} --run \"python Scripts/attraction-repulsion/sonoro_swarm_circ_attr-rep.py\"${NC}"
echo -e "${BOLD}================================================================${NC}"

