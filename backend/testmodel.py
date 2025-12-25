from transformers import pipeline

# Use the exact model ID
model_id = "taskload/bart-cause-effect"

# Create a pipeline for summarization / text2text
cause_effect = pipeline("summarization", model=model_id)

# Test input
event = "The cat chased the mouse and it ran away."
output = cause_effect("text: " + event, max_length=300, min_length=30, do_sample=False)

print("Model output:", output)
