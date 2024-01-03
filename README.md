This project moves the igc intel PCIe card ethernet driver from linux kernel space to user space.
The round trip ethernet send/receive frame cyclic latency reduction is about 1 to 2 orders of magnitude.
The library only supports datalink send/receive packets.

Note this library only works on linux PC's with I225-V ethernet cards.

This project also required the use of acontis atemsys, which provides PCIe memory mapped io, interrupts and dma

Both of the igc and atemsys are licesed under GNU general public license verison 2.0

igc code was forked from linux version 6.6.1
atemesys was forked from version 1.4.25

This library has only been tested with linux version 6.6.1