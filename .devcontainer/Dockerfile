FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

RUN sudo apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    sudo apt-get install --no-install-recommends --assume-yes \
    libx11-dev \
    libgl1 \
    libopenmpi-dev \
    direnv \
    libosmesa6-dev

# Install misc deps
RUN mkdir /home/vscode/scripts
COPY scripts/bootstrap.sh /home/vscode/scripts
RUN chmod +x /home/vscode/scripts/bootstrap.sh
