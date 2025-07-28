FROM python:3.10-slim

RUN apt-get update && apt-get install -y python3-tk libx11-6 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN pip install --no-cache-dir asyncio websockets httpx pillow

CMD ["python", "main_ui.py"]
