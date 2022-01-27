
version: "2.0"
stories:
- story: interactive_story_1
  steps:
  - intent: greet
    entities:
    - username: debora
  - slot_was_set:
    - username: debora
  - action: username_form
  - active_loop: username_form
  - slot_was_set:
    - username: debora
  - slot_was_set:
    - username: debora
  - slot_was_set:
    - requested_slot: null
  - active_loop: null
  - action: utter_greet
  - intent: show
  - action: username_form
  - active_loop: username_form
  - slot_was_set:
    - username: debora
  - slot_was_set:
    - requested_slot: null
  - active_loop: null
  - action: action_show
  - slot_was_set:
    - item_name: null
  - slot_was_set:
    - item_quantity: null
  - intent: goodbye
  - action: utter_goodbye
  - action: action_restart
