# Jetson Setup guide

---
## microSD mount setup / microSD카드 마운트 설정

Insert the microSD card into your Jetson device.
Jetson 보드에 microSD 카드를 삽입합니다.



Run the following command:
다음 명령을 실행합니다.
```bash
lsblk
```

Look for something like `**/dev/mmcblk1p1**`.
`**/dev/mmcblk1p1**`같은 형식의 경로를 찾으세요. *p1까지 쓰여 있어야 함.

---

Get the UUID / UUID 확인

```bash
sudo blkid
```

Copy the UUID of the microSD card partition.
microSD 파티션의 UUID를 복사합니다.

---

Create mount point
마운트 경로 생성

```bash
sudo mkdir -p /mnt/microsd
```

---

Edit `/etc/fstab` 

```bash
sudo gedit /etc/fstab
```

Add the following line at the bottom, replacing `YOUR-UUID` with the actual UUID:
파일 맨 밑에 아래 줄을 추가합니다. `YOUR-UUID` 대신 복사해둔 실제 UUID로 바꾸세요:

```fstab
UUID=YOUR-UUID  /mnt/microsd  ext4  defaults,nofail,x-systemd.automount  0  2
```

---
Test
테스트

```bash
sudo mount -a
df -h | grep microsd
```

If it shows up correctly, the setup is done.
정상적으로 나오면 설정 완료.

---
