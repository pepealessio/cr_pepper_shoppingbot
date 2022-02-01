#!/bin/bash

mkdir -p bakup/${1}

cp -v ./fp_audio/scripts/embeddings_data.pk ./bakup/${1}
cp -v ./ros_chatbot/shopping_bot/database/data.pk ./bakup/${1}
