FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

RUN sudo apt-get update && sudo apt-get install --no-install-recommends --assume-yes \
    libx11-6 \
    libx11-dev \
    libxext6 \
    libxrender1 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    libegl1 \
    libgl1 \
    libglx0 \
    mesa-utils \
    libosmesa6-dev \
    x11-apps \
    direnv \
    curl && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Install gh (GitHub CLI) via the official apt repository.
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
    && chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | tee /etc/apt/sources.list.d/github-cli.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gh \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# for teleops
RUN groupadd -g 104 input && usermod -aG input vscode

# Python dependency manifest — uv reads this to install simulation libs and dev tools.
# /devcontainer is owned by vscode so uv (running as vscode) can write .venv there.
RUN mkdir -p /devcontainer && printf '3.12\n' > /devcontainer/.python-version && chown -R vscode:vscode /devcontainer
COPY --chown=vscode:vscode pyproject.toml /devcontainer/pyproject.toml

# Bootstrap script for standalone Go binaries.
COPY --chown=vscode:vscode scripts/bootstrap.sh /home/vscode/scripts/bootstrap.sh
RUN chmod +x /home/vscode/scripts/bootstrap.sh

WORKDIR /home/vscode
USER vscode
RUN mkdir -p /home/vscode/.local/bin

# Install uv — package and tool manager.
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Activate the project venv system-wide: binaries (ty, ruff) and libs are on PATH
# without any shell activation step.
ENV VIRTUAL_ENV="/devcontainer/.venv"
ENV PATH="/devcontainer/.venv/bin:/home/vscode/.local/bin:${PATH}"

# Install all Python dependencies in one shot:
#   simulation — torch (CPU-only wheel), genesis-world
#   dev tools  — ty (type checker), ruff (formatter/linter)
# CPU-only torch: ~250 MB. Swap the pytorch-cpu index for a CUDA wheel if needed.
RUN uv sync --project /devcontainer

# Install standalone Go binaries: starpls (Bazel LSP), shfmt, buildifier.
RUN /home/vscode/scripts/bootstrap.sh
