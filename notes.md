# Introduction

## You should know (or learn)
- what is an elf file exactly
- how are the files linked, loaded, compiled? 
- what is a free-standing environment, what is a hosted environment?

## ideas
- build your own debugging tools
- multicore
- hardware level memory protection?
- actual virtual memory instead of identity mapping

## Wiki
### RPi Boot Process
1. Videocorm
2. kernel8.img loaded
3. jumps to 0x80000

### newlib
libc implementation, but need to fake some OS services or implement some underlying features for it to work.     

# Assignments

## A0: Polling Loop
- implement a big polling loop
- 