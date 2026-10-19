ARG UBUNTU_DIGEST
FROM ubuntu:24.04@sha256:${UBUNTU_DIGEST}

ARG UBUNTU_DIGEST
ARG RUNNER_VERSION
ARG RUNNER_SHA256

RUN case "$UBUNTU_DIGEST" in \
        ""|*[!0-9a-fA-F]*) echo "UBUNTU_DIGEST must be 64 hexadecimal characters" >&2; exit 1 ;; \
    esac; \
    test "${#UBUNTU_DIGEST}" -eq 64; \
    test -n "$RUNNER_VERSION"; \
    case "$RUNNER_VERSION" in \
        ""|*[!0-9.]*|.*|*.) echo "RUNNER_VERSION must be numeric and dotted" >&2; exit 1 ;; \
    esac; \
    case "$RUNNER_SHA256" in \
        ""|*[!0-9a-fA-F]*) echo "RUNNER_SHA256 must be 64 hexadecimal characters" >&2; exit 1 ;; \
    esac; \
    test "${#RUNNER_SHA256}" -eq 64

RUN set -eux; \
    apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        dosfstools \
        file \
        g++-mingw-w64 \
        g++-multilib \
        gcc-multilib \
        git \
        iptables \
        jq \
        libicu74 \
        libkrb5-3 \
        libssl3t64 \
        mtools \
        openjdk-21-jdk-headless \
        python3 \
        rsync \
        unzip \
        util-linux \
        xxd \
        zip \
        zlib1g; \
    rm -rf /var/lib/apt/lists/*; \
    groupadd --gid 1001 runner; \
    useradd --uid 1001 --gid 1001 --create-home \
        --home-dir /home/runner --shell /bin/bash runner; \
    mkdir -p /opt/actions-runner /opt/ci-bin; \
    curl --fail --location --proto '=https' --tlsv1.2 \
        --output /tmp/actions-runner.tar.gz \
        "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"; \
    printf '%s  %s\n' "$RUNNER_SHA256" /tmp/actions-runner.tar.gz \
        | sha256sum --check --strict -; \
    tar -xzf /tmp/actions-runner.tar.gz -C /opt/actions-runner; \
    rm -f /tmp/actions-runner.tar.gz; \
    chown -R 1001:1001 /opt/actions-runner /home/runner

COPY make-wrapper.sh /opt/ci-bin/make
COPY entrypoint.sh /usr/local/bin/capoworks-runner-entrypoint
RUN chmod 0755 /opt/ci-bin/make /usr/local/bin/capoworks-runner-entrypoint

ENV HOME=/home/runner \
    NUMJOBS=-j2 \
    JOBS=2 \
    JAVA_TOOL_OPTIONS=-Xmx1024m \
    GRADLE_OPTS="-Dorg.gradle.workers.max=2 -Dorg.gradle.daemon=false" \
    PATH=/opt/ci-bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

WORKDIR /opt/actions-runner
ENTRYPOINT ["/usr/local/bin/capoworks-runner-entrypoint"]
CMD ["runner"]
