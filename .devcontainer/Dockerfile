# proudly AI-generated, human-reviewed
#
# Devcontainer image: RBE base + X11/GL, dev tools, and standalone binaries.
# The heavy Python sim deps (torch, genesis-world) come pre-baked in the base image,
# so this layer only adds what the interactive dev environment needs.

FROM ghcr.io/mockasort-studio/adp8-rbe:latest

USER root
RUN apt-get update && apt-get install --no-install-recommends --assume-yes \
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
    curl && \
    apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Install gh (GitHub CLI) via the official apt repository.
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
    && chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | tee /etc/apt/sources.list.d/github-cli.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gh \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# For teleops: add vscode to the input group.
RUN groupadd -g 104 input && usermod -aG input vscode

# Bootstrap script for standalone dev binaries (starpls, shfmt, buildifier).
COPY --chown=vscode:vscode scripts/bootstrap.sh /home/vscode/scripts/bootstrap.sh
RUN chmod +x /home/vscode/scripts/bootstrap.sh

USER vscode

# Add dev tools (ty, ruff) on top of the sim-only venv from the base image.
RUN uv sync --project /devcontainer --extra dev

# Install standalone dev binaries.
RUN /home/vscode/scripts/bootstrap.sh
