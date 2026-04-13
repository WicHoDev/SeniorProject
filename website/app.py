from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def welcome():
    x = {"Message": "Hello World"}
    return x
