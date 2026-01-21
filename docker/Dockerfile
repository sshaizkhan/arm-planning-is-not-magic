FROM ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    cmake \
    pkg-config \
    libompl-dev \
    libeigen3-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
    update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1

# Create workspace directory
WORKDIR /workspace

# Copy requirements first for better caching
COPY requirements.txt pyproject.toml ./

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    if [ -f requirements.txt ]; then \
        pip install --no-cache-dir -r requirements.txt; \
    else \
        pip install --no-cache-dir \
            numpy \
            toppra \
            ruckig \
            matplotlib; \
    fi

# Install OMPL Python bindings
# Try to import from system package first, fallback to pip
RUN python3 -c "import ompl" 2>/dev/null || pip install --no-cache-dir ompl || echo "Warning: OMPL Python bindings not available"

# Copy project files
COPY . /workspace/arm-planning-is-not-magic/

# Set working directory to project root
WORKDIR /workspace/arm-planning-is-not-magic

# Install the package in editable mode
RUN pip install --no-cache-dir -e .

# Set Python path
ENV PYTHONPATH=/workspace/arm-planning-is-not-magic:${PYTHONPATH}

# Default command
CMD ["/bin/bash"]
