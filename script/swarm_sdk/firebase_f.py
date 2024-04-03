import firebase_admin
from firebase_admin import credentials, db

def get_shape():
    # Initialize Firebase Admin SDK
    cred = credentials.Certificate("drones_firebase_credentials.json")
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://drones-telloedu-robophone-default-rtdb.europe-west1.firebasedatabase.app'
    })

    # Reference to the root of your database
    ref = db.reference('/edu-tello')

    shape = None
    try:
        # Get the values
        shape = ref.child('shape').get()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up: close the Firebase app
        firebase_admin.delete_app(firebase_admin.get_app())

    return shape
