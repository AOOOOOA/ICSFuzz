from ubuntu:18.04

# Install base libs
run apt-get update && apt-get install --no-install-recommends -y python3.7 python3.7-dev python3-pip


RUN apt-get install --no-install-recommends -y libpng16-16=1.6.34-1ubuntu0.18.04.2 \
libtiff5 libjpeg8=8c-2ubuntu8 build-essential=12.4ubuntu1 wget=1.19.4-1ubuntu2.2 git \
libxerces-c-dev \
&& rm -rf /var/lib/apt/lists/* 

# # Install python requirements
run pip3 install --user setuptools==46.3.0 

run pip3 install --user wheel==0.34.2

run pip3 install py_trees==0.8.3 networkx==2.2

RUN pip3 install --upgrade pip setuptools wheel

# (Optional) Install additional libraries that might be needed
RUN apt-get update && apt-get install --no-install-recommends -y \
    libssl-dev \
    libffi-dev \
    && rm -rf /var/lib/apt/lists/*

run pip3 install six==1.14.0 numpy==1.18.4 ephem

run pip3 install shapely==1.7.0 xmlschema==1.1.3 tabulate==0.8.7
# && mkdir -p /app/scenario_runner 

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt
# RUN sh requirements.sh 

# Set the working directory in the container
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgtk2.0-dev \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch and torchvision
RUN pip3 install torch torchvision

# Install other dependencies for PyTorch3D
RUN pip3 install fvcore iopath

# Install PyTorch3D
RUN pip3 install "git+https://github.com/facebookresearch/pytorch3d.git"

# Copy the rest of your application code to the container
COPY . /workspace

# Install scenario_runner 
# copy . /app/scenario_runner

# setup environment :
# 
#   CARLA_HOST :    uri for carla package without trailing slash. 
#                   For example, "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
#
#   CARLA_RELEASE : Name of the package to be used. For example, "CARLA_0.9.9".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
# 
#
#  It's expected that $(CARLA_HOST)/$(CARLA_RELEASE).tar.gz is a downloadable resource.
#

env CARLA_HOST ""
env CARLA_RELEASE ""

# Set environment variables from CARLA_VER
RUN export DEFAULT_CARLA_HOST="$(sed -e 's/^\s*HOST\s*=\s*//;t;d' scenario_runner/CARLA_VER)" && \
    export CARLA_HOST="${CARLA_HOST:-$DEFAULT_CARLA_HOST}" && \
    export DEFAULT_CARLA_RELEASE="$(sed -e 's/^\s*RELEASE\s*=\s*//;t;d' /scenario_runner/CARLA_VER)" && \
    export CARLA_RELEASE="${CARLA_RELEASE:-$DEFAULT_CARLA_RELEASE}" && \
    echo "CARLA_HOST=${CARLA_HOST}" >> /etc/environment && \
    echo "CARLA_RELEASE=${CARLA_RELEASE}" >> /etc/environment

env PYTHONPATH "${PYTHONPATH}:carla/agents:/carla"
# entrypoint ["/bin/sh" ]
entrypoint ["sh launch_all_FLV.sh" ]


