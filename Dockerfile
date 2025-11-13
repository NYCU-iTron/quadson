FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Specify terminal color
ENV TERM=xterm-256color

RUN apt-get update

RUN apt-get install -y curl
RUN apt-get install -y git

# Install zsh
RUN apt-get install -y zsh
RUN chsh -s $(which zsh)

# Install oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install p10k
RUN git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
RUN sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="powerlevel10k\/powerlevel10k"/g' ~/.zshrc
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
COPY dotfiles/.zshrc /root/.zshrc

# Copy gitstatus binary
COPY dotfiles/gitstatus /root/.cache/gitstatus

# Install zsh plugins
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' ~/.zshrc

# Install Python 3.10 and pip
RUN apt-get install -y \
    python3.10 \
    python3-pip \
    && ln -sf /usr/bin/python3.10 /usr/bin/python \
    && ln -sf /usr/bin/python3.10 /usr/bin/python3

# Install python packages
COPY requirements-real.txt /root/quadson_py/requirements-real.txt
COPY requirements-sim.txt /root/quadson_py/requirements-sim.txt
RUN pip3 install --no-cache-dir -r /root/quadson_py/requirements-real.txt
RUN pip3 install --no-cache-dir -r /root/quadson_py/requirements-sim.txt

# Dependencies and tools
RUN apt-get install -y build-essential
RUN apt-get install -y tmux
RUN apt-get install -y can-utils
RUN apt-get install -y kmod
RUN apt-get install -y iproute2
RUN apt-get install -y libusb-1.0-0-dev
RUN apt-get install -y libgl1-mesa-glx
RUN apt-get install -y libglib2.0-0

# Copy CAN scripts
COPY bash/start_can.sh /root/quadson_py/bash/start_can.sh
COPY bash/stop_can.sh /root/quadson_py/bash/stop_can.sh
RUN chmod +x /root/quadson_py/bash/*.sh

# Install project files
COPY src/ /root/quadson_py/src/
COPY setup.py /root/quadson_py/setup.py
RUN pip install --no-cache-dir -e /root/quadson_py

WORKDIR /root/quadson_py
