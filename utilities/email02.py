import imaplib 
import email
import time

def checkEmail():
    
    mail = imaplib.IMAP4_SSL('imap.gmail.com')
    mail.login('timenpm701@gmail.com','islipxdkmqckruef')
    mail.list()
    
    count = 0
    
    while count < 60:
        try:
            # Connect to inbox
            mail.select("inbox")
            
            # Search for an unread email from user's email address
            result, data = mail.search(None, '(UNSEEN FROM "tsweene1@umd.edu")')
            
            print(result)
            print(len(data))
            
            ids = data [0]  # data is a list
            id_list = ids.split() # ids is a space separated string
            
            latest_email_id = id_list[-1] # get the latest
            result, data = mail.fetch(latest_email_id, "(RFC822)")
            
            if data is None:
                print("Waiting...")
                
            if data is not None:
                print("Message received!")
                break
                
        except IndexError:
            time.sleep(2)
            if count < 59:
                count = count + 1
                continue
            else:
                print("Gameover")
                count = 60
                
                
checkEmail()
            