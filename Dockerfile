FROM python:3-alpine

RUN apk add --no-cache build-base

WORKDIR /usr/src/app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY ship_sim.py roc_sim.py ./

CMD [ "/bin/sh" ]
