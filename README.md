This project moves the igc intel PCIe card ethernet driver from linux kernel space to user space.
The round trip ethernet send/receive frame cyclic latency reduction is about half.
The library only supports datalink send/receive packets.

Note this library only works on any Linux PC with an Intel ethernet LAN adapter using the igc Intel driver i.e one of the following 
ethernet PCIe cards from Intel (I225_LM, I225_V, I225_I, I220_V, I225_K, I225_K2, I226_K, I225_LMVP, I226_LMVP, I225_IT, I226_LM, I226_V, I226_IT, I221_V).

This project also required the use of acontis atemsys, which provides PCIe memory mapped io, interrupts and dma

Both of the igc and atemsys are licesed under GNU general public license verison 2.0

igc code was forked from linux version 6.6.1
atemesys was forked from version 1.4.25

This library has only been tested with linux version 6.6.1