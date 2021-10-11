# Assignment 2

Hướng dẫn Tiếng Việt tại đây [Notion](https://harsh-fold-50d.notion.site/Assignment-2-16e9320a1ed84cab89f1d745f7a47bcb)

Hướng dẫn Tiếng Anh tại đây [CS184](https://cs184.eecs.berkeley.edu/sp21/docs/proj2)
# Để có thể chạy GUI trên WSL (windows subsystem for linux):

- Thêm 2 dòng sau vào `~/.bashrc`

```
export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf 2>/dev/null):0
export LIBGL_ALWAYS_INDIRECT=0
```

- khởi động lại terminal

- Sử dụng MobaXterm tại đây [download](https://mobaxterm.mobatek.net/download.html)

- kiểm tra xem có bật X11 server chưa

- Chạy file `./meshedit ../bzc/curve1.bzc` để mở cửa sổ GUI
