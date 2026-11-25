# LF-Bot

Du an robot tu hanh bam line va vuot chuong ngai vat cho cuoc thi F-Maze-Solving tai Dai hoc FPT. Repository gom firmware ESP32 (`firmware/`), PCB base 1 mat lam tai nha (`hardware/`) va tai lieu (`docs/`).

## Bat dau

Cai `just` va cac cong cu build theo [huong dan phat trien](AGENTS.md), sau do xem cac tac vu tai thu muc goc:

```sh
just --list
```

Cac lenh thuong dung: `just test` de kiem thu, `just build` de dich firmware ESP32. Danh sach tac vu duoc quan ly trong [justfile](justfile).

## Tai lieu

- [Kien truc](docs/ARCHITECTURE.md), [trang thai](docs/IMPLEMENTATION.md) va [viec con lai](docs/PLAN.md)
- [Quyet dinh kien truc](docs/decisions/README.md)
- [Quy tac dong gop va lam viec voi agent](AGENTS.md)
- [The le](docs/manual.md) va [hoi dap cuoc thi](docs/fqa.csv)
- [BOM phan cung](docs/HARDWARE_BOM.csv)
- [Phan cung module va huong dan dau day](hardware/README.md)
