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
    libgl1-mesa-dri \
    mesa-utils \
    libosmesa6-dev \
    x11-apps \
    libopenmpi-dev \
    direnv \
    curl \
    # for fastdds gen 
    openjdk-17-jre-headless && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# for teleops
RUN groupadd -g 104 input && usermod -aG input vscode

WORKDIR /home/vscode
USER vscode
RUN mkdir -p /home/vscode/.local/bin

RUN NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" && \
    echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> ~/.bashrc


# Add brew and llvm to PATH for Dockerfile RUN steps
ENV PATH="/home/linuxbrew/.linuxbrew/bin: \
    /home/linuxbrew/.linuxbrew/sbin: \
    /home/linuxbrew/.linuxbrew/opt/llvm/bin: \
    /home/linuxbrew/.linuxbrew/opt/binutils/bin:${PATH}"

RUN sudo chown -R vscode:vscode /home/linuxbrew/.linuxbrew
RUN --mount=type=cache,target=/home/vscode/.cache/Homebrew
RUN sudo chown -R vscode /home/vscode

RUN /home/vscode/scripts/bootstrap.sh /workspaces/ADP8

#python is in homebrew folder. to avoid custom shebanging for py_rules we symlink it where bazel is expecting it
RUN sudo ln -sf $(which python3) /usr/bin/python3
