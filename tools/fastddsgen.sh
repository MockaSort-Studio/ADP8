#!/usr/bin/bash

# Color codes
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_section() {
    echo -e "\n${YELLOW}================================${NC}"
    echo -e "${YELLOW}$*${NC}"
    echo -e "${YELLOW}================================${NC}\n"
}

# Main script
log_section "FastDDSGen Execution"
log_info "Running fastddsgen with arguments: $@"
if output=$(/home/vscode/fastddsgen/scripts/fastddsgen "$@" 2>&1); then
    while IFS= read -r line; do
        log_info "$line"
    done <<< "$output"
    log_success "FastDDSGen completed successfully"
else
    while IFS= read -r line; do
        log_error "$line"
    done <<< "$output"
    log_error "FastDDSGen failed with exit code $?"
    exit 1
fi

log_section "FastDDSGen Execution Complete"