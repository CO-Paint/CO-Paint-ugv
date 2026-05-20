FROM python:3.10-slim

WORKDIR /app

COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# /app/src will be mounted via docker-compose
ENV PYTHONPATH=/app/src

# Expose the API port
EXPOSE 8000

# Start script
COPY entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh

CMD ["/app/entrypoint.sh"]
