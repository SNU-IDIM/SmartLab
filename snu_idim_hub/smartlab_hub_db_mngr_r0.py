import sqlite3

class smartlab_hub_db_mngr:
    def __init__(self, database=None):
        self.conn = None
        self.cursor = None

        self.database = database
    
    def Connect(self):
        try:
            self.conn = sqlite3.connect(self.database)
            self.cursor = self.conn.cursor()
        except sqlite3.Error as e:
            print('Error connecting to database.')
    
    def close(self):        
        if self.conn:
            self.conn.commit()
            self.cursor.close()
            self.conn.close()
    
    def __del__(self):
        self.conn.close()

    def get_devices(self):
        query = 'SELECT * FROM devices'
        self.cursor.execute(query)

        return self.cursor.fetchall()
    
    def get_device(self, name):
        query = 'SELECT * FROM devices WHERE Name={0}'.format(name)

        self.cursor.execute(query)

        return self.cursor.fetchall()
    
    def add_device(self, name, type, ip):
        query = 'INSERT INTO devices (Name, Type, IP) VALUES ({0}, {1}, {2})'.format(name, type, ip)
    
        self.cursor.execute(query)
    
    def remove_device(self, name):
        query = 'DELETE FROM devices WHERE name={0}'.format(name)

        self.cursor.execute(query)

if __name__ == '__main__':
    database = 'smartlab_hub_db_r0.db'
    db_mngr = smartlab_hub_db_mngr(database)
    db_mngr.Connect()

    print(db_mngr.get_devices())
    print(db_mngr.get_device("'3DP-1'"))
    db_mngr.add_device("'3DP-5'", 0, "'0.0.0.0:5005'")
    print(db_mngr.get_devices())
    db_mngr.remove_device("'3DP-5'")