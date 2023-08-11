#!/usr/bin/env bash

# fail on unset variables and command errors
set -eu -o pipefail # -x: is for debugging

# Install deno
curl -fsSL https://deno.land/x/install/install.sh | sh
echo "export PATH=\"\${HOME}/.deno/bin:\${PATH}\"" >> ~/.profile
echo "export PATH=\"\${HOME}/.deno/bin:\${PATH}\"" >> ~/.bash_profile

# Install docker compose
DOCKER_COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | jq -r '.tag_name')
mkdir -p "${HOME}/.docker/cli-plugins"
curl -sL "https://github.com/docker/compose/releases/download/${DOCKER_COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o "${HOME}/.docker/cli-plugins/docker-compose"
chmod +x "${HOME}/.docker/cli-plugins/docker-compose"
