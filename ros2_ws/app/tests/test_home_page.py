import pytest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import os
import sys
import subprocess
import socket
from contextlib import closing


class TestHomePageUploads:
    """Test class for home page file upload functionality"""
    
    @pytest.fixture(scope="class")
    def live_server(self):
        """Start Flask app once for all tests"""
        # Since you run tests from app directory, we need to:
        # 1. Go up one level (from app to ros2_ws)
        # 2. Run python3 app/app.py
        parent_dir = os.path.dirname(os.getcwd())  # Go up one level from app to ros2_ws
        
        # Start Flask app
        app_process = subprocess.Popen([
            sys.executable, "app/app.py"
        ], cwd=parent_dir)
        
        # Wait for app to start
        time.sleep(5)
        
        yield "http://127.0.0.1:5000"
        
        # Cleanup
        app_process.terminate()
        app_process.wait()
    
    @pytest.fixture(autouse=True)
    def setup_selenium(self, live_server):
        """Setup Selenium webdriver"""
        # Get home directory
        home_dir = os.path.expanduser("~")
        self.algo_file = os.path.join(home_dir, "a_star_algorithm.py")
        self.map_file = os.path.join(home_dir, "benchmark.txt")
        self.scenario_file = os.path.join(home_dir, "scenario.txt")
        
        # Check if files exist
        if not os.path.exists(self.algo_file):
            pytest.skip("Algorithm file not found")
        if not os.path.exists(self.map_file):
            pytest.skip("Map file not found")
        if not os.path.exists(self.scenario_file):
            pytest.skip("Scenario file not found")
        
        # Set up Selenium
        self.driver = webdriver.Chrome()
        self.driver.implicitly_wait(10)
        self.base_url = live_server
        
        yield
        
        # Cleanup
        self.driver.quit()
    
    def test_upload_algorithm(self):
        """Test uploading a_star_algorithm.py"""
        # Go to home page
        self.driver.get(f"{self.base_url}/home")
        
        # Find the algorithm upload form
        algo_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-algorithm']")
        
        # Upload the file
        file_input = algo_form.find_element(By.CSS_SELECTOR, "input[type='file']")
        file_input.send_keys(self.algo_file)
        
        # Click upload button
        upload_button = algo_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
        upload_button.click()
        
        # Wait for success message
        alert = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CLASS_NAME, "alert"))
        )
        
        assert "uploaded successfully" in alert.text
        
        # Check if file appears in algorithm dropdown
        algo_select = self.driver.find_element(By.CSS_SELECTOR, "select[name='algorithm']")
        options = algo_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        assert "a_star_algorithm.py" in option_texts
    
    def test_upload_map(self):
        """Test uploading benchmark.txt"""
        # Go to home page
        self.driver.get(f"{self.base_url}/home")
        
        # Find the map upload form
        map_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-benchmark']")
        
        # Upload the file
        file_input = map_form.find_element(By.CSS_SELECTOR, "input[type='file']")
        file_input.send_keys(self.map_file)
        
        # Click upload button
        upload_button = map_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
        upload_button.click()
        
        # Wait for success message
        alert = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CLASS_NAME, "alert"))
        )
        
        assert "uploaded successfully" in alert.text
        
        # Check if file appears in map dropdown
        map_select = self.driver.find_element(By.CSS_SELECTOR, "select[name='map']")
        options = map_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        assert "benchmark.txt" in option_texts
    
    def test_upload_scenario(self):
        """Test uploading scenario.txt"""
        # Go to home page
        self.driver.get(f"{self.base_url}/home")
        
        # Find the scenario upload form
        scenario_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-scenario']")
        
        # Upload the file
        file_input = scenario_form.find_element(By.CSS_SELECTOR, "input[type='file']")
        file_input.send_keys(self.scenario_file)
        
        # Click upload button
        upload_button = scenario_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
        upload_button.click()
        
        # Wait for success message
        alert = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CLASS_NAME, "alert"))
        )
        
        assert "uploaded successfully" in alert.text
        
        # Check if file appears in scenario dropdown
        scenario_select = self.driver.find_element(By.CSS_SELECTOR, "select[name='scenario']")
        options = scenario_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        assert "scenario.txt" in option_texts


    def test_algorithm_dropdown_has_default_options(self):
        """Test that algorithm dropdown has builtin options"""
        self.driver.get(f"{self.base_url}/home")
        
        algo_select = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "select[name='algorithm']"))
        )
        options = algo_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        # Check for builtin algorithms
        assert "Algorithm A" in option_texts
        assert "Algorithm B" in option_texts
        assert "Algorithm C" in option_texts

    def test_map_dropdown_has_default_options(self):
        """Test that map dropdown has builtin options"""
        self.driver.get(f"{self.base_url}/home")
        
        map_select = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "select[name='map']"))
        )
        options = map_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        # Check for builtin map
        assert "map1" in option_texts

    def test_scenario_dropdown_has_default_options(self):
        """Test that scenario dropdown has builtin options"""
        self.driver.get(f"{self.base_url}/home")
        
        scenario_select = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "select[name='scenario']"))
        )
        options = scenario_select.find_elements(By.TAG_NAME, "option")
        option_texts = [option.text for option in options]
        
        # Check for builtin scenario
        assert "scen1" in option_texts

    def test_dropdowns_are_selectable(self):
        """Test that dropdown options can be selected"""
        from selenium.webdriver.support.ui import Select
        self.driver.get(f"{self.base_url}/home")
        
        # Test algorithm dropdown selection
        algo_select_element = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "select[name='algorithm']"))
        )
        algo_select = Select(algo_select_element)
        algo_select.select_by_visible_text("Algorithm A")
        assert algo_select.first_selected_option.text == "Algorithm A"
        
        # Test map dropdown selection
        map_select_element = self.driver.find_element(By.CSS_SELECTOR, "select[name='map']")
        map_select = Select(map_select_element)
        map_select.select_by_visible_text("map1")
        assert map_select.first_selected_option.text == "map1"
        
        # Test scenario dropdown selection
        scenario_select_element = self.driver.find_element(By.CSS_SELECTOR, "select[name='scenario']")
        scenario_select = Select(scenario_select_element)
        scenario_select.select_by_visible_text("scen1")
        assert scenario_select.first_selected_option.text == "scen1"


    # def test_simulate_with_uploaded_files(self):
    #     """Test simulation with uploaded files"""
    #     from selenium.webdriver.support.ui import Select
    #     import subprocess
        
    #     # Go to home page
    #     self.driver.get(f"{self.base_url}/home")
        
    #     # Upload algorithm file
    #     algo_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-algorithm']")
    #     algo_file_input = algo_form.find_element(By.CSS_SELECTOR, "input[type='file']")
    #     algo_file_input.send_keys(self.algo_file)
    #     algo_upload_button = algo_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
    #     algo_upload_button.click()
        
    #     # Wait for algorithm upload success
    #     WebDriverWait(self.driver, 10).until(
    #         EC.presence_of_element_located((By.CLASS_NAME, "alert"))
    #     )
        
    #     # Upload map file
    #     self.driver.get(f"{self.base_url}/home")  # Refresh page
    #     map_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-benchmark']")
    #     map_file_input = map_form.find_element(By.CSS_SELECTOR, "input[type='file']")
    #     map_file_input.send_keys(self.map_file)
    #     map_upload_button = map_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
    #     map_upload_button.click()
        
    #     # Wait for map upload success
    #     WebDriverWait(self.driver, 10).until(
    #         EC.presence_of_element_located((By.CLASS_NAME, "alert"))
    #     )
        
    #     # Upload scenario file
    #     self.driver.get(f"{self.base_url}/home")  # Refresh page
    #     scenario_form = self.driver.find_element(By.CSS_SELECTOR, "form[action='/upload-scenario']")
    #     scenario_file_input = scenario_form.find_element(By.CSS_SELECTOR, "input[type='file']")
    #     scenario_file_input.send_keys(self.scenario_file)
    #     scenario_upload_button = scenario_form.find_element(By.CSS_SELECTOR, "button[type='submit']")
    #     scenario_upload_button.click()
        
    #     # Wait for scenario upload success
    #     WebDriverWait(self.driver, 10).until(
    #         EC.presence_of_element_located((By.CLASS_NAME, "alert"))
    #     )
        
    #     # Refresh page to see all uploaded files in dropdowns
    #     self.driver.get(f"{self.base_url}/home")
        
    #     # Select uploaded algorithm
    #     algo_select_element = WebDriverWait(self.driver, 10).until(
    #         EC.presence_of_element_located((By.CSS_SELECTOR, "select[name='algorithm']"))
    #     )
    #     algo_select = Select(algo_select_element)
    #     algo_select.select_by_visible_text("a_star_algorithm.py")
        
    #     # Select uploaded map
    #     map_select_element = self.driver.find_element(By.CSS_SELECTOR, "select[name='map']")
    #     map_select = Select(map_select_element)
    #     map_select.select_by_visible_text("benchmark.txt")
        
    #     # Select uploaded scenario
    #     scenario_select_element = self.driver.find_element(By.CSS_SELECTOR, "select[name='scenario']")
    #     scenario_select = Select(scenario_select_element)
    #     scenario_select.select_by_visible_text("scenario.txt")
        
    #     # Find the simulate button and scroll to it
    #     simulate_button = WebDriverWait(self.driver, 10).until(
    #         EC.presence_of_element_located((By.CSS_SELECTOR, "form[action='/simulate'] button[type='submit']"))
    #     )
        
    #     # Scroll to the button
    #     self.driver.execute_script("arguments[0].scrollIntoView();", simulate_button)
    #     time.sleep(1)  # Wait a moment after scrolling
        
    #     # Make sure the button is clickable
    #     simulate_button = WebDriverWait(self.driver, 10).until(
    #         EC.element_to_be_clickable((By.CSS_SELECTOR, "form[action='/simulate'] button[type='submit']"))
    #     )
        
    #     # Click the button
    #     simulate_button.click()
        
    #     # Wait for simulation start message
    #     alert = WebDriverWait(self.driver, 15).until(
    #         EC.presence_of_element_located((By.CLASS_NAME, "alert"))
    #     )
        
    #     # Verify simulation started
    #     assert "Simulation started" in alert.text
    #     assert "a_star_algorithm.py" in alert.text
    #     assert "benchmark.txt" in alert.text
    #     assert "scenario.txt" in alert.text
        
    #     # Wait 10 seconds for simulation to run
    #     time.sleep(10)
        
    #     # Kill all ROS2 and Gazebo related processes
    #     subprocess.run(['pkill', '-INT', '-f', 'ros2'], check=False)
    #     subprocess.run(['pkill', '-INT', '-f', 'gazebo'], check=False)
    #     subprocess.run(['pkill', '-INT', '-f', 'gzserver'], check=False)
    #     subprocess.run(['pkill', '-INT', '-f', 'gzclient'], check=False)
        
    #     # Test passes if we get here
    #     assert True

class TestHomePageNavigation:
    """Test navigation links on the home page"""
    
    @pytest.fixture(scope="class")
    def live_server(self):
        """Start Flask app once for all navigation tests"""
        parent_dir = os.path.dirname(os.getcwd())  # Go up one level from app to ros2_ws
        
        # Start Flask app
        app_process = subprocess.Popen([
            sys.executable, "app/app.py"
        ], cwd=parent_dir)
        
        # Wait for app to start
        time.sleep(5)
        
        yield "http://127.0.0.1:5000"
        
        # Cleanup
        app_process.terminate()
        app_process.wait()
    
    @pytest.fixture(autouse=True)
    def setup_selenium(self, live_server):
        """Setup Selenium webdriver"""
        self.driver = webdriver.Chrome()
        self.driver.implicitly_wait(10)
        self.base_url = live_server
        
        yield
        
        self.driver.quit()
    
    def test_navigation_links_work(self):
        """Test all navigation menu links work from home page"""
        # Go to home page
        self.driver.get(f"{self.base_url}/home")
        
        # Test navigation links
        nav_tests = [
            ("Main", "/"),
            ("Home", "/home"),
            ("Instructions", "/info"),
            ("About", "/about"),
        ]
        
        for link_text, expected_url_part in nav_tests:
            # Find and click navigation link
            nav_link = WebDriverWait(self.driver, 10).until(
                EC.element_to_be_clickable((By.LINK_TEXT, link_text))
            )
            nav_link.click()
            
            # Wait for navigation
            WebDriverWait(self.driver, 10).until(
                EC.url_contains(expected_url_part)
            )
            
            # Verify we're on the expected page
            assert expected_url_part in self.driver.current_url, f"Failed navigation to {link_text}"
            
            # Go back to home page for next test (unless we're already there)
            if expected_url_part != "/home":
                self.driver.get(f"{self.base_url}/home")
    
    def test_logo_links_to_main_page(self):
        """Test that clicking the logo navigates to main page"""
        # Go to home page
        self.driver.get(f"{self.base_url}/home")
        
        # Click on the logo/brand link
        logo_link = WebDriverWait(self.driver, 10).until(
            EC.element_to_be_clickable((By.CSS_SELECTOR, "header a"))
        )
        logo_link.click()
        
        # Wait for navigation to main page
        WebDriverWait(self.driver, 10).until(
            EC.url_matches(f"{self.base_url}/$")
        )
        
        # Verify we're on the main page
        assert self.driver.current_url == f"{self.base_url}/"
    
    def test_page_title_correct(self):
        """Test that the home page has correct title"""
        self.driver.get(f"{self.base_url}/home")
        assert self.driver.title == "MAPF Simulator"
    
    def test_navigation_header_exists(self):
        """Test that the navigation header with correct elements exists"""
        self.driver.get(f"{self.base_url}/home")
        
        # Check that header exists
        header = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.TAG_NAME, "header"))
        )
        
        # Check for logo image
        logo_img = header.find_element(By.CSS_SELECTOR, "img")
        assert "icons8-robot-2-30.png" in logo_img.get_attribute("src")
        
        # Check for RoboSim text
        brand_text = header.find_element(By.ID, "headerMain")
        assert brand_text.text == "RoboSim"
        
        # Check that all navigation links are present
        nav_links = header.find_elements(By.CSS_SELECTOR, ".nav-link")
        nav_link_texts = [link.text for link in nav_links]
        
        expected_links = ["Main", "Home", "Instructions", "About"]
        for expected_link in expected_links:
            assert expected_link in nav_link_texts