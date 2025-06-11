from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.common.exceptions import WebDriverException, NoAlertPresentException, TimeoutException
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.chrome.options import Options
import random


class TwotruckWebDriver:
    def __init__(self):
        self.options = Options()
        self.options.add_argument("--start-fullscreen")
        self.driver = webdriver.Chrome(options=self.options)
        self.username = 'admin2'
        self.password = 'admin'
        self.agv_name = 'AGV2'
        self.agv_num = 2
        self.curr_mission = None

        self.nav_xpath = {
            "Home": '/html/body/div/div/section/div/ul/li[1]/a',
            "Mission": '/html/body/div/div/section/div/ul/li[2]/a',
            "Truck": '/html/body/div/div/section/div/ul/li[3]/a',
            "Statistics": '/html/body/div/div/section/div/ul/li[4]/a',
            "Battery": '/html/body/div/div/section/div/ul/li[5]/a',
            "Alarm": '/html/body/div/div/section/div/ul/li[6]/a[1]',
            "Login": '/html/body/div/div/section/div/ul/li[7]/a'
        }

        self.pick_xpath = {
            "P01S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[1]',
            "P02S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[2]',
            "P03S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[3]',
            "P04S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[4]',
            "P05S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[5]',
            "P06S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[6]',
            "Back": '/html/body/div/div/section/section/div[2]/div/div[1]/button[9]',
            "Send": '/html/body/div/div/section/section/div[2]/div/div[2]/div/button'
        }

        self.drop_xpath = {
            "D01S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[1]',
            "D02S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[2]',
            "D03S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[3]',
            "D04S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[4]',
            "D05S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[5]',
            "D06S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[6]',
            "D07S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[7]',
            "D08S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[8]',
            "D09S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[9]',
            "D10S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[10]',
            "D11S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[11]',
            "D12S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[12]',
            "D13S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[13]',
            "D14S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[14]',
            "D15S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[15]',
            "D16S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[16]',
            "D17S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[17]',
            "D18S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[18]',
            "D19S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[19]',
            "D20S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[20]',
            "D21S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[21]',
            "D22S": '/html/body/div/div/section/section/div[2]/div/div[1]/button[22]',
            "Send": '/html/body/div/div/section/section/div[2]/div/div[2]/div/button',
            "Back": '/html/body/div/div/section/section/div[2]/div/div[2]/button',
            "Delete": '/html/body/div/div/section/section/div[2]/div/div[2]/div/div[2]/div[2]/div[2]/div[3]/svg/path',
            "Confirm": '/html/body/div/div/section/section/div[1]/div/button'
        }

    def get_website(self):
        self.driver.get("http://192.168.1.14:5010/login")
        self.wait_for_element('//input[@type="text"]')

    def wait_for_element(self, xpath, timeout=10, clickable=False):
        wait = WebDriverWait(self.driver, timeout)
        if clickable:
            wait.until(EC.element_to_be_clickable((By.XPATH, xpath)))
        else:
            wait.until(EC.presence_of_element_located((By.XPATH, xpath)))

    def login(self):
        self.wait_for_element('/html/body/div/div/section/section/section/div[3]/form/div[1]/input')
        username_input = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/div[1]/input')
        password_input = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/div[2]/input')
        username_input.send_keys(self.username)
        password_input.send_keys(self.password)

        login_button = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/form/button[1]')
        login_button.click()

        try:
            WebDriverWait(self.driver, 5).until(EC.alert_is_present())
            alert = self.driver.switch_to.alert
            print("Alert found:", alert.text)
            alert.accept()
            print("Alert accepted.")
        except TimeoutException:
            print("No alert appeared.")
        except NoAlertPresentException:
            print("No alert present exception.")

    def get_navbar(self):
        return {name: self.driver.find_element(By.XPATH, xpath) for name, xpath in self.nav_xpath.items()}

    def create_mission(self):
        # create_btn_xpath = '/html/body/div/div/section/section/section[3]/div[2]/section[2]/div[3]/button'
        create_btn_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
        self.wait_for_element(create_btn_xpath, clickable=True)
        create_mission_btn = self.driver.find_element(By.XPATH, create_btn_xpath)
        create_mission_btn.click()
        pick_point = self.random_points(data_dict=self.pick_xpath)
        self.wait_for_element(self.pick_xpath[pick_point], clickable=True)
        pickup = self.driver.find_element(By.XPATH, self.pick_xpath[pick_point])
        pickup.click()
        self.wait_for_element(self.pick_xpath["Send"], clickable=True)
        send = self.driver.find_element(By.XPATH, self.pick_xpath["Send"])
        send.click()
        self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
        confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
        confirm.click()

    def choose_drop_off(self):
        choose_drop_path = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
        choose_drop_path = '/html/body/div/div/section/section/section[3]/div[2]/section[2]/div[3]/button'
        self.wait_for_element(choose_drop_path, clickable=True)
        # drop_points = self.random_points(data_dict=self.drop_xpath, type='drop')
        drop_points = ['D01S', 'D03S', 'D06S', 'D12S']
        for drop in drop_points:
            try:
                self.wait_for_element(self.drop_xpath[drop], clickable=True)
                drop_button = self.driver.find_element(By.XPATH, self.drop_xpath[drop])
                drop_button.click()
            except Exception as err:
                print(f'Error: {err}')
        self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
        confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
        confirm.click()

    def drop_product(self):
        drop_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[3]/button'
        drop_product = self.driver.find_element(By.XPATH, drop_xpath)
        drop_product.click()
        self.wait_for_element('/html/body/div/div/section/section/div[1]/div/button', clickable=True)
        confirm = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/div[1]/div/button')
        confirm.click()

    def stop_vehicle(self):
        stop_xpath = f'/html/body/div/div/section/section/section[3]/div[2]/section[{self.agv_num}]/div[1]/div[2]/button'
        stop_button = self.driver.find_element(By.XPATH, stop_xpath)
        stop_button.click()

    def continue_vehicle(self):
        continue_xpath = f'/html/body/div/div/section/section/div[{self.agv_num}]/div/button'
        continue_button = self.driver.find_element(By.XPATH, continue_xpath)
        continue_button.click()

    def logout(self):
        self.wait_for_element('/html/body/div/div/section/section/section/div[3]/div/button')
        logout_button = self.driver.find_element(By.XPATH, '/html/body/div/div/section/section/section/div[3]/div/button')
        logout_button.click()

    def close_website(self):
        self.driver.quit()
    
    def random_points(self,data_dict, type=''):
        if type == 'drop':
            count_num = random.randint(1,4) #if random number between 1-4
            return random.sample(list(data_dict.keys()), count_num)
        else:
            return random.choice(list(data_dict.keys()))

    def wait_until_user_closes_browser(self):
        try:
            while True:
                if len(self.driver.window_handles) == 0:
                    break
        except WebDriverException:
            pass

def main():
    driver = TwotruckWebDriver()
    driver.get_website()
    print("Browser is open. Close the window to end the program.")
    driver.login()
    navbar = driver.get_navbar()
    navbar['Home'].click()
    driver.wait_for_element(driver.nav_xpath["Home"])
    # driver.create_mission()
    driver.choose_drop_off()
    # driver.drop_product()
    driver.wait_until_user_closes_browser()
    print("Browser closed. Program exiting.")

if __name__ == "__main__":
    main()
