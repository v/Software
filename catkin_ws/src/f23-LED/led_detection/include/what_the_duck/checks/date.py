from what_the_duck.check import CheckFailed

class CheckDate():
    def __init__(self):
        pass
    
    def check(self):
        import datetime
        now = datetime.datetime.now()
        if now.year < 2017:
            msg = 'The date is not set correctly. This will screw up building.'
            raise CheckFailed(msg)
         

