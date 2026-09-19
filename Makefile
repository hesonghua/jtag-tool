# jtag_tool — EBAZ4205 板上 JTAG 调试工具（py 参考实现 + C 性能版）
#
#   make            # 交叉编译（动态链 buildroot staging 的 readline）→ overlay
#   make host       # 本机 gcc 自检编译（无 readline，仅查语法）
#   make clean
#
# 链接说明：动态链接 buildroot staging 里的 libreadline/libhistory/libc——
# 与板端 rootfs 同源同版本（ABI/glibc 符号完全匹配），板上已有这些 .so
# （python3-readline 带进来的）。host 上没有 arm readline，所以 host 目标
# 不链 readline（folds back to fgets）。

# 直接用 buildroot 的外部工具链（编板端 rootfs 的同一支 gcc）——
# libc/crt/glibc 符号版本天然匹配板端（Ubuntu 交叉 gcc 的 crt 会引入
# GLIBC_2.34 符号，板上 2.33 跑不了）
CC      := ../build/rootfs/host/bin/arm-none-linux-gnueabihf-gcc
CFLAGS  := -O2 -g -Wall -Wextra -std=gnu11

STAGING := ../build/rootfs/staging
# readline/history 从 buildroot staging 取（同工具链产物）
RL_CFLAGS := -I$(STAGING)/usr/include
RL_LDFLAGS := -L$(STAGING)/usr/lib -lreadline -lhistory

OVERLAY_BIN := ../overlay/root

all: install

install: $(OVERLAY_BIN)/jtag_tool $(OVERLAY_BIN)/jtag_tool.py

$(OVERLAY_BIN)/jtag_tool: jtag_tool.c
	@mkdir -p $(OVERLAY_BIN)
	$(CC) $(CFLAGS) $(RL_CFLAGS) -o $@ $< $(RL_LDFLAGS)
	@ls -la $@

$(OVERLAY_BIN)/jtag_tool.py: jtag_tool.py
	@mkdir -p $(OVERLAY_BIN)
	cp $< $@

host:
	cc $(CFLAGS) -DNO_READLINE -o jtag_tool_host jtag_tool.c

clean:
	rm -f jtag_tool_host

.PHONY: all install host clean
