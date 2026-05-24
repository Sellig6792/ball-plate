Utilise ces commandes pour lier la caméra à WSL (si tu utilises WSL)
```shell
usbipd list // Choisis le BUS_ID correspondant à la caméra souhaitée
usbipd bind <BUS_ID>
usbipd attach --wsl -ab <BUS_ID>
```

