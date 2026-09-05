FROM python:3.13-slim

WORKDIR /app

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

COPY pyproject.toml uv.lock README.md ./
COPY src ./src
RUN uv sync --frozen --no-dev

COPY templates ./templates
COPY shared ./shared

EXPOSE 8000

ENTRYPOINT ["uv", "run", "spore-control-plane"]
