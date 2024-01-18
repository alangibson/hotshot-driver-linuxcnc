FROM debian:bookworm

RUN apt update && \
    apt install -y curl build-essential gdb
RUN curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-amd64/linuxcnc-uspace_2.9.1_amd64.deb
RUN curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-amd64/linuxcnc-uspace-dev_2.9.1_amd64.deb
RUN curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-amd64/linuxcnc-uspace-dbgsym_2.9.1_amd64.deb
RUN dpkg -i linuxcnc-uspace_2.9.1_amd64.deb linuxcnc-uspace-dev_2.9.1_amd64.deb linuxcnc-uspace-dbgsym_2.9.1_amd64.deb || true
RUN apt --fix-broken -y install
