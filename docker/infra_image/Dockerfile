# This is an adapted file by DAS to geneate a 
ARG FROM_IMAGE=ubuntu:bionic
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
  bash-completion \  
  python3-pip



# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  nginx-full \
  mosquitto \
  openssh-server \
  sudo \
  && rm -rf /var/lib/apt/lists/*
# Set root password
RUN    echo 'root:password' >> /root/passwdfile
# Create user and it's password
RUN    useradd -m -G sudo master && \
  echo 'master:password' >> /root/passwdfile
# Apply root password
RUN    chpasswd -c SHA512 < /root/passwdfile && \
  rm /root/passwdfile &&  chmod -R 777 /var/www && mkdir /var/run/sshd
CMD service nginx restart && service ssh restart && mosquitto
