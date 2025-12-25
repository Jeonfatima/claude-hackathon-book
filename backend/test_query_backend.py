import requests
import json
import os

# Get the actual API key from environment or use a default
api_key = os.getenv("REQUIRED_API_KEY", "test-api-key")

url = "http://127.0.0.1:8000/query"
headers = {
    "Content-Type": "application/json",
    "X-API-Key": api_key
}
data = {
    "question": "What is ROS 2?",
    "selected_text": ""
}

try:
    response = requests.post(url, headers=headers, json=data, timeout=180)
    print("Status code:", response.status_code)
    print("Response JSON:", response.json())
    if response.status_code == 200 and "answer" in response.json():
        print("PASS: Backend returned an answer!")
    else:
        print("FAIL: No answer or server error.")
except requests.exceptions.Timeout:
    print("FAIL: Request timed out. Gemini API may be slow or unavailable.")
except Exception as e:
    print("FAIL:", str(e))