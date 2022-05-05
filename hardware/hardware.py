"""
File to interface between the python scripts and hardware
"""

class Hardware:
    def __init__(self):
        self.desktop = 'address' #address of desktop computer
        self.remote = 'address 2' #address of remote computer on bicycle

    def get_state(self):
        """
        Function to get the full state vector from the hardware.
        This will be sent from the arduino (remote) to the desktop.
        """
        return
    
    def send_input(self):
        """
        Function to send full state vector from desktop to remote at eah time step
        """
        return