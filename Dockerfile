# lidar-racer -- reproducible build and test environment.
#
# What is in the image: the AI, its tests, and the shell script that wires it
# to the simulator API.
#
# What is deliberately not in the image: CoppeliaSim and the n4s API binaries.
# Neither is ours to redistribute, CoppeliaSim needs a GPU and a display, and
# an image that shipped a 2GB simulator would take longer to pull than to
# compile the thing it is testing. The container talks to a CoppeliaSim
# running on the host instead -- see docs/07-docker.md.

# ---------------------------------------------------------------------------
# Build stage: compile and run the test suite. A failing test fails the build,
# so an image that exists is an image whose tests passed.
# ---------------------------------------------------------------------------
FROM debian:bookworm-slim AS build

RUN apt-get update \
    && apt-get install -y --no-install-recommends gcc libc6-dev make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY Makefile ./
COPY include/ ./include/
COPY src/ ./src/
COPY tests/ ./tests/

RUN make all && make test

# ---------------------------------------------------------------------------
# Runtime stage: just enough to run the pipeline.
# ---------------------------------------------------------------------------
FROM debian:bookworm-slim AS runtime

# bash for the pipeline script, coreutils for mkfifo. Nothing else is needed:
# the AI is a static-libc C binary that speaks a text protocol over pipes.
RUN apt-get update \
    && apt-get install -y --no-install-recommends bash coreutils \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --shell /bin/bash racer

WORKDIR /app
COPY --from=build /src/ai /app/ai
COPY scripts/ /app/scripts/
COPY docs/ /app/docs/
COPY README.md /app/

RUN chmod +x /app/scripts/run.sh && chown -R racer:racer /app
USER racer

# The API binaries are bind-mounted here at run time rather than baked in.
ENV N4S=/opt/n4s/n4s
ENV BUFFER=/opt/n4s/n4s_buffer
ENV AI=/app/ai

ENTRYPOINT ["/app/scripts/run.sh"]
