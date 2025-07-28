FROM python:3.10

WORKDIR /app
COPY . .

RUN pip install --no-cache-dir fastapi pydantic uvicorn httpx requests websockets

CMD ["python", "main_fms.py"]