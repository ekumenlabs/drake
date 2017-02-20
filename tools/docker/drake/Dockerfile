FROM ubuntu:xenial
MAINTAINER Agustin Alba Chicar <ag.albachicar@ekumenlabs.com>


# Setup environment
ENV DEBIAN_FRONTEND noninteractive
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# Install development packages
RUN apt-get update && apt-get install -y sudo \
	git \
	build-essential \
	mercurial \
	vim \
	tmux \
	openssh-server \
	software-properties-common \
	bash-completion \
	debian-keyring \
	debian-archive-keyring

# Create a user with passwordless sudo
RUN adduser --gecos "Development User" --disabled-password drake
RUN adduser drake sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/drake/.bashrc

# Install Drake dependencies
RUN apt-get install -y --no-install-recommends lsb-core software-properties-common wget
RUN wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | apt-key add -
RUN add-apt-repository -y "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main"
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        clang-3.9 \
        lldb-3.9 \
		autoconf \
		automake \
		bash-completion \
		bison \
		clang-format \
		default-jdk \
		doxygen \
		flex \
		freeglut3-dev \
		g++ \
		g++-5 \
		g++-5-multilib \
		gdb \
		gfortran \
		gfortran-5 \
		gfortran-5-multilib \
		git \
		graphviz \
		libgl1-mesa-dri \
		libgtk2.0-dev \
		libhtml-form-perl \
		libmpfr-dev \
		libpng12-dev \
		libqt4-dev \
		libqt4-opengl-dev \
		libqwt-dev \
		libtool \
		libvtk-java \
		libvtk5-dev \
		libvtk5-qt4-dev \
		libxmu-dev \
		make \
		ninja-build \
		perl \
		pkg-config \
		python-bs4 \
		python-dev \
		python-gtk2 \
		python-html5lib \
		python-lxml \
		python-numpy \
		python-scipy \
		python-sphinx \
		python-vtk \
		python-yaml \
		unzip \
		valgrind \
		xvfb \
		zip \
		zlib1g-dev

RUN wget -O /tmp/bazel_0.4.3-linux-x86_64.deb https://github.com/bazelbuild/bazel/releases/download/0.4.3/bazel_0.4.3-linux-x86_64.deb && \
    dpkg -i /tmp/bazel_0.4.3-linux-x86_64.deb

RUN apt-get install --no-install-recommends -y cmake && \
	apt-get install --no-install-recommends -y cmake-curses-gui

#-----------------------------------------------------------------------------------
# Create workspace directory
RUN mkdir -p /home/drake

#-----------------------------------------------------------------------------------
# Install Intel graphic card drivers
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx libgl1-mesa-dri xserver-xorg-video-intel

#-----------------------------------------------------------------------------------
# Change user to drake
USER drake

#-----------------------------------------------------------------------------------
# Download and install Drake from Ekumenlabs fork
RUN mkdir -p /home/drake/drake && \
    cd /home/drake/drake && \
    git clone https://github.com/ekumenlabs/drake.git drake-distro

RUN mkdir /home/drake/drake/drake-distro/build && \
	cd /home/drake/drake/drake-distro/build && \
	cmake .. && \
	make -j8

#-----------------------------------------------------------------------------------
# Set the Drake home directory as the starting point and run tmux
WORKDIR /home/drake
CMD tmux
