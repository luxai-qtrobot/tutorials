#!/bin/bash
# This script installs Ollama on Qtrobot Jetson AGX orin 

OLLAMA_VERSION="v0.3.6"

TEMP_DIR=$(mktemp -d)
OLLAMA_DOWNLOAD_URL="https://github.com/ollama/ollama/releases/download/${OLLAMA_VERSION}/ollama-linux-arm64"
OLLAMA_INSTALL_DIR="/usr/local/bin"


# check if the script is running using sudo
if [ $(id -u) -ne 0 ]
  then echo "This script requires superuser permissions. Please re-run with sudo."
  exit
fi



# download and install ollama binary 
echo
echo "*** Downloading ollama ${OLLAMA_VERSION}..."
curl --fail --show-error --location --progress-bar -o "$TEMP_DIR/ollama" "${OLLAMA_DOWNLOAD_URL}"

echo
echo "*** Installing ollama ${OLLAMA_VERSION} in ${OLLAMA_INSTALL_DIR}"
install -o0 -g0 -m755 $TEMP_DIR/ollama $OLLAMA_INSTALL_DIR/ollama



# Adding Ollama as a startup service
echo
echo "*** Adding Ollama as a startup service..."
if ! id ollama >/dev/null 2>&1; then
    echo "    creating ollama user..."
    useradd -r -s /bin/false -U -m -d /usr/share/ollama ollama
fi
if getent group render >/dev/null 2>&1; then
    echo "    Adding ollama user to render group..."
    usermod -a -G render ollama
fi
if getent group video >/dev/null 2>&1; then
    echo "    Adding ollama user to video group..."
    usermod -a -G video ollama
fi

echo "    adding current user to ollama group..."
usermod -a -G ollama $(whoami)

echo "    creating ollama systemd service..."
cat <<EOF | tee /etc/systemd/system/ollama.service >/dev/null
[Unit]
Description=Ollama Service
After=network-online.target

[Service]
ExecStart=$OLLAMA_INSTALL_DIR/ollama serve
User=ollama
Group=ollama
Restart=always
RestartSec=3
Environment="PATH=$PATH"

[Install]
WantedBy=default.target
EOF
SYSTEMCTL_RUNNING="$(systemctl is-system-running || true)"
case $SYSTEMCTL_RUNNING in
    running|degraded)
        echo "    enabling and starting ollama service..."
        systemctl daemon-reload
        systemctl enable ollama

        start_service() { systemctl restart ollama; }
        trap start_service EXIT
        ;;
esac

echo
echo '**** Install complete. You can run "ollama" from the command line.'

