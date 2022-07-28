# How to install vscode on Jetpack 5.0

- [How to install vscode on Jetpack 5.0](#how-to-install-vscode-on-jetpack-50)

Newest version of vscode from microsoft isn't compatible with the newest version of Jetpack yet. 

An older version works ok:

Instruction from https://forums.developer.nvidia.com/t/vs-code-can-t-launch-with-jetpack-5-0/213980/11

```bash
wget https://update.code.visualstudio.com/1.50.0/linux-deb-arm64/stable -O stable.deb
sudo dpkg -i stable.deb
```