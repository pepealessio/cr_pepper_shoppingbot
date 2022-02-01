#!/bin/bash

rm -v ./fp_audio/scripts/embeddings_data.pk
rm -v ./ros_chatbot/shopping_bot/database/data.pk
cp -v ./bakup/${1}/embeddings_data.pk ./fp_audio/scripts/embeddings_data.pk
cp -v ./bakup/${1}/data.pk ./ros_chatbot/shopping_bot/database/data.pk
