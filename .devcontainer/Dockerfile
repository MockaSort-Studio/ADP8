FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

# Install misc deps
RUN mkdir /home/vscode/scripts
COPY scripts/bootstrap.sh /home/vscode/scripts
COPY scripts/.Brewfile /home/vscode
RUN chown -R vscode:vscode /home/vscode/
RUN chmod +x /home/vscode/scripts/bootstrap.sh

# for teleops
RUN groupadd -g 104 input && usermod -aG input vscode

WORKDIR /home/vscode
USER vscode

RUN mkdir -p .local/bin
RUN mkdir -p .ssh && chmod 700 .ssh

RUN ssh-keyscan -t ecdsa github.com >> .ssh/known_hosts

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
    git-lfs \
    curl \
    # for fastdds gen to be later moved directly in bazel workspace by pushing the compiled artifacts separately and pulling the binary
    openjdk-17-jre 

# Download and install fastdds-gen
RUN NONINTERACTIVE=1 /bin/bash -c "git clone --recursive --branch v4.2.0 --single-branch https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen" && \
    cd fastddsgen && ./gradlew assemble


RUN NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" && \
    echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> ~/.bashrc && sudo chown -R vscode:vscode /home/linuxbrew/.linuxbrew


# Add brew and llvm to PATH for Dockerfile RUN steps
ENV PATH="/home/linuxbrew/.linuxbrew/bin: \
    /home/linuxbrew/.linuxbrew/sbin: \
    /home/linuxbrew/.linuxbrew/opt/llvm/bin: \
    /home/linuxbrew/.linuxbrew/opt/binutils/bin:${PATH}"

# Install git-lfs
RUN git lfs install
