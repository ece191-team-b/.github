# How to use the new `nvfancontrol` daemon for Jetback version >= 5.0 

- [How to use the new `nvfancontrol` daemon for Jetback version >= 5.0](#how-to-use-the-new-nvfancontrol-daemon-for-jetback-version--50)
  - [Nvidia Documentation](#nvidia-documentation)
  - [TLDR](#tldr)
    - [Config file](#config-file)
    - [To change the default (quiet) fan profile](#to-change-the-default-quiet-fan-profile)

## Nvidia Documentation

https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html?highlight=fan#fan-profile-control

## TLDR

### Config file

```bash
cat /etc/nvfancontrol.conf # this is where the fan config is stored
```

### To change the default (quiet) fan profile 

```bash
sudo systemctl stop nvfancontrol # stop service
```

Change this line in `/etc/nvfancontrol.conf`

```bash
FAN_DEFAULT_PROFILE <fan_profile> # quiet / cool
FAN_DEFAULT_PROFILE cool # you probably want to do this
```
Then, 
```bash
sudo rm /var/lib/nvfancontrol/status # remove status file
sudo systemctl start nvfancontrol #restart service
```