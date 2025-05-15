# tests/test_info_page.py
import pytest
from app import app
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import threading
import socket
from contextlib import closing


# Shared fixture for the Flask app
@pytest.fixture(scope="class")
def live_server():
    """Start Flask app once for all navigation tests"""
    # Find free port
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.listen(1)
        port = s.getsockname()[1]
    
   
    server_thread = threading.Thread(
        target=lambda: app.run(host='127.0.0.1', port=port, debug=False, use_reloader=False)
    )
    server_thread.daemon = True
    server_thread.start()
    time.sleep(2)
    yield f"http://127.0.0.1:{port}"


class TestInfoPageRoutes:
    """Test the info page route and basic functionality"""
    
    @pytest.fixture
    def client(self):
        """Create a test client for the Flask app"""
        app.config['TESTING'] = True
        with app.test_client() as client:
            yield client
    
    def test_info_route_loads(self, client):
        """Test that the info route '/info' loads successfully"""
        response = client.get('/info')
        assert response.status_code == 200
    
    def test_info_route_has_correct_title(self, client):
        """Test that the page has the correct title"""
        response = client.get('/info')
        assert b'<title>MAPF Simulator</title>' in response.data
    
    def test_info_page_has_main_sections(self, client):
        """Test that all main sections exist"""
        response = client.get('/info')
        assert b'Upload Files' in response.data
        assert b'Algorithms File' in response.data
        assert b'Map/Benchmark File' in response.data
        assert b'Scenario File' in response.data
        assert b'Select Files' in response.data
    
    def test_info_page_has_navigation_header(self, client):
        """Test that the navigation header exists"""
        response = client.get('/info')
        assert b'RoboSim' in response.data
        assert b'nav' in response.data


class TestInfoPageContent:
    """Test the content and structure of the info page"""
    
    @pytest.fixture(autouse=True)
    def setup_selenium(self, live_server):
        """Setup Selenium webdriver"""
        self.driver = webdriver.Chrome()
        self.driver.implicitly_wait(5)
        self.base_url = live_server
        
        yield
        self.driver.quit()
    
    def test_page_title_in_browser(self):
        """Test that the browser tab shows correct title"""
        self.driver.get(f"{self.base_url}/info")
        assert self.driver.title == "MAPF Simulator"
    
    def test_navigation_links_work(self):
        """Test all navigation menu links work from info page"""
        self.driver.get(f"{self.base_url}/info")
        
        nav_tests = [
            ("Main", "/"),
            ("Home", "/home"),
            ("About", "/about")
        ]
        
        for link_text, expected_url_part in nav_tests:
            nav_link = WebDriverWait(self.driver, 10).until(
                EC.element_to_be_clickable((By.LINK_TEXT, link_text))
            )
            nav_link.click()
            
            time.sleep(1)
            
            assert expected_url_part in self.driver.current_url, f"Failed navigation to {link_text}"
            
            # Go back to info page for next test
            self.driver.get(f"{self.base_url}/info")
    
    def test_all_section_headings_present(self):
        """Test that all main section headings are present"""
        self.driver.get(f"{self.base_url}/info")
        
        expected_headings = [
            "Upload Files",
            "Algorithms File",
            "Map/Benchmark File",
            "Scenario File",
            "Select Files"
        ]
        
        for heading in expected_headings:
            heading_element = WebDriverWait(self.driver, 10).until(
                EC.presence_of_element_located((By.XPATH, f"//h1[contains(text(), '{heading}')]"))
            )
            assert heading_element.is_displayed()
    
    def test_all_section_images_present(self):
        """Test that all section images load correctly"""
        self.driver.get(f"{self.base_url}/info")
        
        expected_images = [
            "upload view.png",
            "algorithm.png",
            "map (2).png",
            "scenario.png",
            "select.png"
        ]
        
        for image_name in expected_images:
            image = WebDriverWait(self.driver, 10).until(
                EC.presence_of_element_located((By.CSS_SELECTOR, f"img[src*='{image_name}']"))
            )
            assert image.is_displayed()
    
    def test_code_examples_present(self):
        """Test that code examples are properly displayed"""
        self.driver.get(f"{self.base_url}/info")
        
        # Check for code tags
        code_elements = self.driver.find_elements(By.TAG_NAME, "code")
        assert len(code_elements) >= 3 
        assert any("def algo(benchmark: str, scenario: str)" in elem.text for elem in code_elements)
        assert any(".py" in elem.text for elem in code_elements)
        assert any(".txt" in elem.text for elem in code_elements)
    
    def test_external_links_work(self):
        """Test that external links to movingai.com are present"""
        self.driver.get(f"{self.base_url}/info")
        
        # Find links to the external website
        external_links = self.driver.find_elements(By.CSS_SELECTOR, "a[href*='movingai.com']")
        assert len(external_links) >= 2 
        
        # Verify the links have correct href
        for link in external_links:
            assert "https://movingai.com/benchmarks/mapf/index.html" in link.get_attribute("href")
    
    def test_logo_image_exists(self):
        """Test that the robot logo exists in the header"""
        self.driver.get(f"{self.base_url}/info")
        
        logo = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "header img"))
        )
        
        assert "icons8-robot-2-30.png" in logo.get_attribute("src")
    
    def test_bootstrap_styles_applied(self):
        """Test that Bootstrap CSS classes are properly applied"""
        self.driver.get(f"{self.base_url}/info")
        
        # Check for container classes
        containers = self.driver.find_elements(By.CLASS_NAME, "container")
        assert len(containers) >= 5  # Each section has a container
        
        # Check for Bootstrap grid classes
        rows = self.driver.find_elements(By.CLASS_NAME, "row")
        assert len(rows) >= 5  # Each section has a row
        
        # Check for rounded corners and shadows
        rounded_elements = self.driver.find_elements(By.CLASS_NAME, "rounded-3")
        shadow_elements = self.driver.find_elements(By.CLASS_NAME, "shadow-lg")
        assert len(rounded_elements) >= 5
        assert len(shadow_elements) >= 5