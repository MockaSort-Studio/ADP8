FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

# Install misc deps
RUN mkdir /home/vscode/scripts
COPY scripts/bootstrap.sh /home/vscode/scripts
COPY scripts/.Brewfile /home/vscode
RUN chown -R vscode:vscode /home/vscode/
RUN chmod +x /home/vscode/scripts/bootstrap.sh

WORKDIR /home/vscode
USER vscode

RUN mkdir -p .local/bin
RUN mkdir -p .ssh && chmod 700 .ssh

RUN ssh-keyscan -t ecdsa github.com >> .ssh/known_hosts

RUN sudo apt-get update
RUN sudo apt-get install --no-install-recommends --assume-yes \
    libx11-dev \
    libgl1 \
    libopenmpi-dev \
    direnv \
    libosmesa6-dev \
    x11-apps \
    git-lfs \
    curl

RUN NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" && \
    echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> ~/.bashrc && sudo chown -R vscode:vscode /home/linuxbrew/.linuxbrew


# Add brew and llvm to PATH for Dockerfile RUN steps
ENV PATH="/home/linuxbrew/.linuxbrew/bin: \
    /home/linuxbrew/.linuxbrew/sbin: \
    /home/linuxbrew/.linuxbrew/opt/llvm/bin: \
    /home/linuxbrew/.linuxbrew/opt/binutils/bin:${PATH}"

# Install git-lfs
RUN git lfs install

# for teleops
RUN groupadd -g 104 input && usermod -aG input vscode