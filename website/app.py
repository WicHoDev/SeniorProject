from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def welcome():
    x = "Test, Hello"
    return x
