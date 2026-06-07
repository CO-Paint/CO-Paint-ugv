FROM python:3.10-slim

WORKDIR /app

COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt

COPY src/ /app/src/
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV PYTHONPATH=/app

EXPOSE 8000

CMD ["/app/entrypoint.sh"]
