# 32Raven build image.
# Provides the STM32 ARM toolchain, uv, and ESP-IDF host prerequisites.
# ESP-IDF tools themselves are installed at runtime via `make idf-install`,
# which writes into a persisted volume (.docker/home/.espressif on the host).
FROM ubuntu:24.04

# uv resolves every first-party Python script's inline (PEP 723) dependencies at
# build time, so kconfiglib/jinja2/pillow are NOT apt-installed here — the
# scripts declare what they need and uv fetches it. Pinned for reproducibility.
# The cache lands in $HOME (a persisted volume), so only the first build in a
# fresh checkout downloads anything.
COPY --from=ghcr.io/astral-sh/uv:0.11.26 /uv /uvx /usr/local/bin/

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      gcc-arm-none-eabi \
      binutils-arm-none-eabi \
      libnewlib-arm-none-eabi \
      libstdc++-arm-none-eabi-newlib \
      cmake \
      ninja-build \
      make \
      git \
      python3 \
      python3-pip \
      python3-venv \
      python3-setuptools \
      wget \
      curl \
      flex \
      bison \
      gperf \
      ccache \
      libffi-dev \
      libssl-dev \
      dfu-util \
      libusb-1.0-0 \
      ripgrep \
      clang-format \
      clang-tidy \
      ca-certificates \
      bash \
      bash-completion \
      less \
      nano \
      vim \
    && rm -rf /var/lib/apt/lists/*

# Non-root user. Makefile-driven runs override UID/GID via `-u` (or
# --userns=keep-id on rootless podman); the VSCode Dev Containers extension
# uses `updateRemoteUserUID` to remap this user to the host UID at attach time.
ARG USERNAME=builder
ARG USER_UID=1000
ARG USER_GID=1000
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME

USER $USERNAME
WORKDIR /workspace
