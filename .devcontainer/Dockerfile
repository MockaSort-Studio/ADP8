FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04
# Install misc deps
RUN mkdir /home/vscode/scripts
COPY scripts/install-deps.sh /home/vscode/scripts
RUN chmod +x /home/vscode/scripts/install-deps.sh
