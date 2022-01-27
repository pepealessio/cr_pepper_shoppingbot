#!/bin/bash
BOT_DIR=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "${BOT_DIR}/../shopping_bot"

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api

