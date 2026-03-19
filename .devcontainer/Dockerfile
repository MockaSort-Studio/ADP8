FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04
# Install misc deps
RUN mkdir /home/vscode/scripts
COPY scripts/bootstrap.sh /home/vscode/scripts
COPY scripts/.Brewfile /home/vscode
RUN chown -R vscode:vscode /home/vscode/
RUN chmod +x /home/vscode/scripts/bootstrap.sh /home/vscode/.Brewfile

RUN sudo apt-get update
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
    python3-pip \
    curl && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Simulation packages — owned by the container image, not by Bazel's pip hubs.
# torch is installed first so genesis-world's import check passes at install time.
# CPU-only torch: ~250 MB. Swap index URL for a CUDA wheel if GPU support is needed.
# python3-pip is a prerequisite; the rest of the system Python is untouched.
RUN pip3 install --break-system-packages --no-cache-dir \
    torch --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --break-system-packages --no-cache-dir \
    genesis-world

# for teleops
RUN groupadd -g 104 input && usermod -aG input vscode



WORKDIR /home/vscode
USER vscode
RUN mkdir -p /home/vscode/.local/bin

RUN NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" && \
    echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> ~/.bashrc


# Add brew and llvm to PATH for Dockerfile RUN steps
ENV PATH="/home/linuxbrew/.linuxbrew/bin:/home/linuxbrew/.linuxbrew/sbin:${PATH}"

RUN sudo chown -R vscode:vscode /home/linuxbrew/.linuxbrew
RUN --mount=type=cache,target=/home/vscode/.cache/Homebrew
RUN sudo chown -R vscode /home/vscode

RUN /home/vscode/scripts/bootstrap.sh /workspaces/ADP8


