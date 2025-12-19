#!/bin/bash
# Project Cleanup and Organization Script
# Ensures the project structure is clean and properly organized

set -e

echo "🧹 Cleaning up LiDAR-Camera Fusion project..."

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if we're in the right directory
if [[ ! -f "README.md" ]] || [[ ! -d "src" ]]; then
    log_error "Please run this script from the project root directory"
    exit 1
fi

# Clean Python cache files
log_info "Cleaning Python cache files..."
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
find . -name "*.pyc" -delete 2>/dev/null || true
find . -name "*.pyo" -delete 2>/dev/null || true

# Check for misplaced files
log_info "Checking file organization..."

# Files that should be in src/infrastructure/
infrastructure_files=(
    "async_pipeline.py"
    "memory_pool.py"
    "rolling_buffer.py"
    "test_infrastructure.py"
)

for file in "${infrastructure_files[@]}"; do
    if [[ -f "$file" ]] && [[ ! -f "src/infrastructure/$file" ]]; then
        log_warn "Moving $file to src/infrastructure/"
        mv "$file" "src/infrastructure/"
    fi
done

# Files that should be in docs/
docs_files=(
    "optimization_issues.py"
)

for file in "${docs_files[@]}"; do
    if [[ -f "$file" ]] && [[ ! -f "docs/$file" ]]; then
        log_warn "Moving $file to docs/"
        mv "$file" "docs/"
    fi
done

# Remove old/unnecessary files
old_files=(
    "start.sh"  # Replaced by scripts/setup_ubuntu_24.sh
)

for file in "${old_files[@]}"; do
    if [[ -f "$file" ]]; then
        log_warn "Removing old file: $file"
        rm "$file"
    fi
done

# Check file permissions
log_info "Checking file permissions..."

# Make scripts executable
find scripts/ -name "*.sh" -exec chmod +x {} \;

# Make Python files readable
find . -name "*.py" -exec chmod 644 {} \;

# Check for required directories
required_dirs=(
    "src/core"
    "src/infrastructure"
    "src/utils"
    "tests/hardware_tests"
    "config"
    "docs"
    "scripts"
)

for dir in "${required_dirs[@]}"; do
    if [[ ! -d "$dir" ]]; then
        log_error "Missing required directory: $dir"
        exit 1
    fi
done

# Validate Python syntax (not runtime imports)
log_info "Validating Python syntax..."

python_files=$(find . -name "*.py" -not -path "./__pycache__/*" -not -path "./.*")

error_count=0
for file in $python_files; do
    if ! python3 -m py_compile "$file" 2>/dev/null; then
        log_error "Syntax error in $file"
        ((error_count++))
    fi
done

if [[ $error_count -gt 0 ]]; then
    log_error "Found $error_count files with syntax errors"
    exit 1
else
    log_info "All Python files have valid syntax"
fi

# Check for syntax errors
log_info "Checking Python syntax..."

python_files=$(find . -name "*.py" -not -path "./__pycache__/*" -not -path "./.*")

error_count=0
for file in $python_files; do
    if ! python3 -m py_compile "$file" 2>/dev/null; then
        log_error "Syntax error in $file"
        ((error_count++))
    fi
done

if [[ $error_count -gt 0 ]]; then
    log_error "Found $error_count files with syntax errors"
    exit 1
fi

# Generate project summary
log_info "Generating project summary..."

total_files=$(find . -type f -not -path "./__pycache__/*" -not -path "./.*" | wc -l)
total_lines=$(find . -name "*.py" -o -name "*.sh" -o -name "*.md" | xargs wc -l | tail -1 | awk '{print $1}')
python_files=$(find . -name "*.py" -not -path "./__pycache__/*" | wc -l)

echo ""
echo "📊 Project Summary:"
echo "  Total files: $total_files"
echo "  Total lines: $total_lines"
echo "  Python files: $python_files"

# Directory breakdown
echo ""
echo "📁 Directory Structure:"
echo "  src/core/           - Core fusion algorithms"
echo "  src/infrastructure/ - Real-time optimizations"
echo "  src/utils/          - Utilities and tools"
echo "  tests/              - Test suites"
echo "  scripts/            - Setup and deployment"
echo "  config/             - Configuration files"
echo "  docs/               - Documentation"

echo ""
echo "✅ Project cleanup complete!"
echo ""
echo "🚀 Ready for development and deployment!"
echo ""
echo "Next steps:"
echo "  1. Hardware setup: ./scripts/setup_ubuntu_24.sh"
echo "  2. Run tests: python3 tests/test_system.py"
echo "  3. Start fusion: python3 src/core/fusion.py"
